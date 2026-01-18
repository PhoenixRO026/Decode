package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.commonlibs.units.Pose
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.library.buttons.ButtonReader
import org.firstinspires.ftc.teamcode.robot.Robot
import java.util.concurrent.TimeUnit

@TeleOp
class BoringDrive : LinearOpMode(){
    @Config
    data object BoringDriveConfig {
        @JvmField var ticksPerRev = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0)
        @JvmField var pos = ticksPerRev / 3.0
        @JvmField var multiplier = 0
        @JvmField var shooterOffset = 94.0
        @JvmField var intakeOffset = 0.0
        @JvmField var shooterTargetRpm = 3300
    }

    var intakeAction : Action? = null

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))
        val timeKeep = TimeKeep()
        var lastPos : Boolean = false // false = intake true = shooter

        val intakeRight = ButtonReader { gamepad2.y}
        val intakeLeft = ButtonReader { gamepad2.a}
        val shootRight = ButtonReader { gamepad2.b}
        val shootLeft = ButtonReader { gamepad2.x}
        val fingerUp = ButtonReader {gamepad2.dpad_up}
        val fingerDown = ButtonReader {gamepad2.dpad_down}
        val highRpm = ButtonReader {gamepad2.right_bumper}
        val lowRpm = ButtonReader {gamepad2.left_bumper}
        val stopShooter = ButtonReader {gamepad2.dpad_left}
        val dpadRight = ButtonReader {gamepad2.dpad_right}
        val buttons = listOf(intakeRight, intakeLeft, shootRight, shootLeft, fingerUp, fingerDown, highRpm, lowRpm, stopShooter, dpadRight)


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
            robot.drive.updatePoseEstimate()

            /// Drive

            if (gamepad1.y) {
                robot.drive.resetFieldCentric()
            }
            robot.drive.isSlowMode = gamepad1.right_trigger >= 0.2
            robot.drive.driveFieldCentric(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble()
            )

            /// Transfer
            if (fingerUp.wasJustPressed())
                robot.transfer.fingerUp()

            if (fingerDown.wasJustPressed())
                robot.transfer.fingerDown()

            if (intakeRight.wasJustPressed()){
                BoringDriveConfig.multiplier--
                robot.transfer.goToPos(BoringDriveConfig.pos, BoringDriveConfig.multiplier, BoringDriveConfig.intakeOffset)
                lastPos = false
            }

            if (intakeLeft.wasJustPressed()){
                BoringDriveConfig.multiplier++
                robot.transfer.goToPos(BoringDriveConfig.pos, BoringDriveConfig.multiplier, BoringDriveConfig.intakeOffset)
                lastPos = false
            }

            if (shootRight.wasJustPressed()){
                if(lastPos) {
                    BoringDriveConfig.multiplier--
                }
                robot.transfer.goToPos(BoringDriveConfig.pos, BoringDriveConfig.multiplier,BoringDriveConfig.shooterOffset)
                lastPos = true
            }
            if (shootLeft.wasJustPressed()){
                if (lastPos) {
                    BoringDriveConfig.multiplier++
                }
                robot.transfer.goToPos(BoringDriveConfig.pos, BoringDriveConfig.multiplier,BoringDriveConfig.shooterOffset)
                lastPos = true
            }

            /// Intake


            if (highRpm.wasJustPressed()){ /// shoot far
                robot.shooter.goToRmp(BoringDriveConfig.shooterTargetRpm.toDouble())
            }
            else if (lowRpm.wasJustPressed()) { /// shoot close
                robot.shooter.goToRmp(2500.0)
            }
            else if (stopShooter.wasJustPressed()) { /// stop shoot
                robot.shooter.goToRmp(0.0)
            }

            robot.shooter.updateRpm(timeKeep.deltaTime)



            if(dpadRight.wasJustPressed()) {
                intakeAction = robot.intakeBalls(0,0)
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


            runActions()
            telemetry.addData("Best Match", robot.camera.sensorColor.closestSwatch)
            /*telemetry.addData("ball1", robot.camera.ball1)
            telemetry.addData("ball2", robot.camera.ball2)
            telemetry.addData("ball3", robot.camera.ball3)
            telemetry.addData("a was pressed (set)", gamepad1.a)
            telemetry.addData("x was pressed (left)", gamepad1.x)
            telemetry.addData("b was pressed (right)", gamepad1.b)
            telemetry.addData("up was pressed (set)", gamepad1.dpad_up)
            telemetry.addData("down was pressed (left)", gamepad1.dpad_down)
            telemetry.addData("shooter power", robot.shooter.power)
            telemetry.addData("rpm", robot.shooter.rpm)
            telemetry.addData("target rpm", robot.shooter.targetRpm)*/
            telemetry.addData("pos", robot.transfer.position)
            //telemetry.addData("fingir pos", robot.transfer.finger.position)
            telemetry.addData("target pos", robot.transfer.targetPosition)
            telemetry.addData("power trans", robot.transfer.power)
            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.addData("multiplier", BoringDriveConfig.multiplier)
            telemetry.addData("action", intakeAction)
            telemetry.update()

            robot.transfer.update(timeKeep.deltaTime)
        }
    }

    private fun runActions() {
        intakeAction?.let {
            val p = TelemetryPacket()
            if (!it.run(p)) {
                intakeAction = null
            }
            FtcDashboard.getInstance().sendTelemetryPacket(p)
        }
    }

}