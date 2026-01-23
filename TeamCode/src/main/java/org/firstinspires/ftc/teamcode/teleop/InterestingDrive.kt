package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Trajectory
import com.commonlibs.units.Pose
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.commonlibs.units.inch
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.library.buttons.ButtonReader
import org.firstinspires.ftc.teamcode.library.buttons.ToggleButtonReader
import org.firstinspires.ftc.teamcode.robot.LimeLightCore.LimeLightConfig.headingToleranceDeg
import org.firstinspires.ftc.teamcode.robot.Robot
import java.util.concurrent.TimeUnit

@TeleOp
class InterestingDrive : LinearOpMode(){
    @Config
    data object InterestingDriveConfig {
        @JvmField var ticksPerRev = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0)
        @JvmField var pos = ticksPerRev / 3.0
        @JvmField var multiplier = 0
        @JvmField var shooterOffset = 94.0
        @JvmField var intakeOffset = 0.0
        @JvmField val rpmFar = 3260.0
        @JvmField val rpmClose = 2700.0
    }

    val smallTrianglePose = Pose(55.inch, -10.inch, 203.0.deg)

    var intakeAction : Action? = null
    var shootAction : Action? = null

    var driveAction : Action? = null

    var currTrajectory: Trajectory? = null
    var goingToTarget = false

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))
        val timeKeep = TimeKeep()
        var shootOffset : Boolean = false // false = intake true = shooter

        robot.limelight.setPipeline(1)

        val shootRight = ButtonReader { gamepad2.b}
        val shootLeft = ButtonReader { gamepad2.x}
        val highRpm = ButtonReader {gamepad2.right_bumper}
        val lowRpm = ButtonReader {gamepad2.left_bumper}
        val stopShooter = ButtonReader {gamepad2.dpad_left}
        val dpadRight = ButtonReader {gamepad2.dpad_right}
        val spew = ButtonReader {gamepad1.left_trigger >= 0.2}
        val stopButton = ButtonReader {gamepad2.touchpad}
        val snipe = ToggleButtonReader {gamepad1.x}


        val buttons = listOf(shootRight, shootLeft, highRpm, lowRpm, stopShooter, dpadRight, spew, stopButton, snipe)


        robot.transfer.finger.position = 1.0

        while (opModeInInit()){
            robot.camera.portal.getProcessorEnabled(robot.camera.colorSensor)
        }

        val exposureCtrl = robot.camera.portal.getCameraControl(ExposureControl::class.java)
        exposureCtrl.setExposure(CameraConfig.desiredExposureMs, TimeUnit.MILLISECONDS)

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            buttons.forEach { it.readValue() }

            // regular manual control

            if (gamepad1.y) {
                robot.drive.resetFieldCentric()
            }
            robot.drive.isSlowMode = gamepad1.right_trigger >= 0.2

            if (snipe.state) {
                robot.limelight.driveWithHeading(
                    -gamepad1.left_stick_y.toDouble(),
                    -gamepad1.left_stick_x.toDouble(),
                    timeKeep.deltaTime
                )
            } else {
                robot.drive.driveFieldCentric(
                    -gamepad1.left_stick_y.toDouble(),
                    -gamepad1.left_stick_x.toDouble(),
                    -gamepad1.right_stick_x.toDouble()
                )
            }


                /// Transfer

            if (shootRight.wasJustPressed()) {
                if (shootOffset) {
                    InterestingDriveConfig.multiplier--
                    if (InterestingDriveConfig.multiplier == -1) {
                        InterestingDriveConfig.multiplier = 2
                    }
                }
                robot.transfer.goToPos(
                    InterestingDriveConfig.pos,
                    InterestingDriveConfig.multiplier,
                    InterestingDriveConfig.shooterOffset
                )
                shootOffset = true
            }
            if (shootLeft.wasJustPressed()) {
                if (shootOffset) {
                    InterestingDriveConfig.multiplier++
                    if (InterestingDriveConfig.multiplier == 3) {
                        InterestingDriveConfig.multiplier = 0
                    }
                }
                robot.transfer.goToPos(
                    InterestingDriveConfig.pos,
                    InterestingDriveConfig.multiplier,
                    InterestingDriveConfig.shooterOffset
                )
                shootOffset = true
            }

            /// Shooter

            if (highRpm.wasJustPressed()) {
                shootAction = robot.shootTeleBalls(
                    InterestingDriveConfig.rpmFar,
                    InterestingDriveConfig.multiplier
                )
            } else if (lowRpm.wasJustPressed()) {
                shootAction = robot.shootTeleBalls(
                    InterestingDriveConfig.rpmClose,
                    InterestingDriveConfig.multiplier
                )
            } else if (stopShooter.wasJustPressed()) { /// stop shoot
                robot.shooter.goToRmp(0.0)
            }

            if (gamepad1.dpad_left) {
                robot.transfer.power = -0.1
            } else if (gamepad1.dpad_right) {
                robot.transfer.power = 0.1
            } else {
                robot.transfer.power = 0.0
            }

            if (gamepad1.dpad_up) {
                robot.transfer.resetPos()
            }


            /// Intake

            if (dpadRight.wasJustPressed()) {
                intakeAction = robot.intakeTeleBalls(0)
                shootOffset = false
            } else if (spew.wasJustPressed()) {
                intakeAction = robot.intake.spew()
            }

            /// Stop

            if (stopButton.wasJustPressed()) {
                shootAction = null
                intakeAction = null
            }
            runActions()
            telemetry.addData("heading error", robot.limelight.headingErrorDeg)
            telemetry.addData("heading button", snipe)
            telemetry.addData("Best Match", robot.camera.sensorColor.closestSwatch)
            telemetry.addData("pos", robot.transfer.position)
            telemetry.addData("target pos", robot.transfer.targetPosition)
            telemetry.addData("power trans", robot.transfer.power)
            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.addData("multiplier", InterestingDriveConfig.multiplier)
            telemetry.addData("action intake: ", intakeAction)
            telemetry.addData("action drive: ", driveAction)
            telemetry.update()

            robot.shooter.update(timeKeep.deltaTime)
            robot.transfer.update(timeKeep.deltaTime)
        }
    }

    fun runActions() {
        intakeAction?.let {
            val p = TelemetryPacket()
            if (!it.run(p)) {
                intakeAction = null
            }
            FtcDashboard.getInstance().sendTelemetryPacket(p)
        }

        shootAction?.let {
            val p = TelemetryPacket()
            if (!it.run(p)) {
                shootAction = null
            }
            FtcDashboard.getInstance().sendTelemetryPacket(p)
        }
    }
}