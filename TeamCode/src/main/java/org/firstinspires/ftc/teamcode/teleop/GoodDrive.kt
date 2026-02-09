package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.commonlibs.units.Pose
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.library.buttons.ButtonReader
import org.firstinspires.ftc.teamcode.library.buttons.ToggleButtonReader
import org.firstinspires.ftc.teamcode.robot.Robot

@TeleOp
open class GoodDrive : LinearOpMode(){
    @Config
    data object GoodDriveConfing {
        @JvmField var ticksPerRev = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0)
        @JvmField var pos = ticksPerRev / 3.0
        @JvmField var multiplier = 0
        @JvmField var shooterOffset = 94.0
        @JvmField var intakeOffset = 0.0
        @JvmField var rpmSmall = 3260
        @JvmField var rpmBig = 2775
    }
    open val pipeline: Int = 1

    private var driver1Action: Action? = null

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))
        val timeKeep = TimeKeep()
        var lastPos : Boolean = false // false = intake true = shooter

        robot.limelight.setPipeline(pipeline)

        val intakeRight = ButtonReader { gamepad2.y}
        val intakeLeft = ButtonReader { gamepad2.a}
        val shootRight = ButtonReader { gamepad2.b}
        val shootLeft = ButtonReader { gamepad2.x}
        val fingerUp = ButtonReader {gamepad2.dpad_up}
        val fingerDown = ButtonReader {gamepad2.dpad_down}
        val highRpm = ButtonReader {gamepad2.right_bumper}
        val lowRpm = ButtonReader {gamepad2.left_bumper}
        val stopShooter = ButtonReader {gamepad2.dpad_left}
        val snipe = ToggleButtonReader ({gamepad1.x})
        //val autoRpm = ButtonReader {gamepad2.dpad_right}
        val autoIntake = ButtonReader {gamepad2.dpad_right}
        val shootBalls = ButtonReader{gamepad2.right_trigger >= 0.2}
        val intakeBalls = ToggleButtonReader({gamepad2.left_trigger >= 0.2})
        val buttons = listOf(shootBalls, intakeBalls, intakeRight, intakeLeft, shootRight, shootLeft, fingerUp, fingerDown, highRpm, lowRpm, stopShooter, snipe, autoIntake)
        val stopButton = ButtonReader {gamepad2.touchpad}

        robot.transfer.finger.position = 0.9

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            buttons.forEach { it.readValue() }
            robot.drive.updatePoseEstimateOdo()

            /// Drive

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
            if (gamepad1.y) {
                robot.drive.resetFieldCentric()
            }

            /// Transfer

            if (shootBalls.wasJustPressed())
                driver1Action=  robot.shootTeleBalls(3280.0, GoodDriveConfing.multiplier)

            if (fingerUp.wasJustPressed())
                robot.transfer.fingerUp()
            if (fingerDown.wasJustPressed())
                robot.transfer.fingerDown()
            if (intakeBalls.state) {
                if (driver1Action == null) {
                    driver1Action = SequentialAction(
                        robot.intakeTeleBalls(GoodDriveConfing.multiplier),
                        InstantAction { intakeBalls.setState(false) },
                        InstantAction { GoodDriveConfing.multiplier+= 2 }
                    )
                }
            }
            else {
                /// Intake
                if (gamepad1.right_bumper) {
                    robot.intake.power = 1.0
                }
                else if (gamepad1.left_bumper) {
                    robot.intake.power = -1.0
                }
                else {
                    robot.intake.power = 0.0
                }
                if (intakeRight.wasJustPressed()) {
                    GoodDriveConfing.multiplier--
                    robot.transfer.goToPos(
                        GoodDriveConfing.pos,
                        GoodDriveConfing.multiplier,
                        GoodDriveConfing.intakeOffset
                    )
                    lastPos = false
                }

                if (intakeLeft.wasJustPressed()) {
                    GoodDriveConfing.multiplier++
                    robot.transfer.goToPos(
                        GoodDriveConfing.pos,
                        GoodDriveConfing.multiplier,
                        GoodDriveConfing.intakeOffset
                    )
                    lastPos = false
                }

                if (shootRight.wasJustPressed()) {
                    if (lastPos) {
                        GoodDriveConfing.multiplier--
                    }
                    robot.transfer.goToPos(
                        GoodDriveConfing.pos,
                        GoodDriveConfing.multiplier,
                        GoodDriveConfing.shooterOffset
                    )
                    lastPos = true
                }
                if (shootLeft.wasJustPressed()) {
                    if (lastPos) {
                        GoodDriveConfing.multiplier++
                    }
                    robot.transfer.goToPos(
                        GoodDriveConfing.pos,
                        GoodDriveConfing.multiplier,
                        GoodDriveConfing.shooterOffset
                    )
                    lastPos = true
                }
            }



            var rpm = 0.0
            if (highRpm.wasJustPressed()){ /// shoot far
                robot.shooter.goToRmp(GoodDriveConfing.rpmSmall.toDouble())
            }
            else if (lowRpm.wasJustPressed()) { /// shoot close
                robot.shooter.goToRmp(GoodDriveConfing.rpmBig.toDouble())
            }
            else if (autoIntake.wasJustPressed()) {
                robot.intakeTeleBalls(GoodDriveConfing.multiplier)
            }
            else if (stopShooter.wasJustPressed()) { /// stop shoot
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

            robot.shooter.update(timeKeep.deltaTime)

            telemetry.addData("distance", robot.limelight.getDistance())
            telemetry.addData("auto rpm", rpm)

            //telemetry.addData("target heading", robot.limelight.)
            telemetry.addData("error heading", robot.limelight.headingErrorDeg)
            telemetry.addData("a was pressed (set)", gamepad1.a)
            telemetry.addData("x was pressed (left)", gamepad1.x)
            telemetry.addData("b was pressed (right)", gamepad1.b)
            telemetry.addData("up was pressed (set)", gamepad1.dpad_up)
            telemetry.addData("down was pressed (left)", gamepad1.dpad_down)
            telemetry.addData("shooter power", robot.shooter.power)
            telemetry.addData("rpm", robot.shooter.rpm)
            telemetry.addData("target rpm", robot.shooter.targetRpm)
            telemetry.addData("pos", robot.transfer.position)
            telemetry.addData("fingir pos", robot.transfer.finger.position)
            telemetry.addData("target pos", robot.transfer.targetPosition)
            telemetry.addData("power trans", robot.transfer.power)
            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.addData("multiplier", GoodDriveConfing.multiplier)
            telemetry.update()

            robot.transfer.update(timeKeep.deltaTime)
            runActions()
        }
    }

    private fun runActions() {
        driver1Action?.let {
            if (!it.run(TelemetryPacket())) {
                driver1Action = null
            }
        }
    }
}