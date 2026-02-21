package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
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
import org.firstinspires.ftc.teamcode.robot.Spindexer.BallColor


@TeleOp
open class BlindDriveAutohead : LinearOpMode(){
    open val pip: Int = 1
    @Config
    data object BlindDrive {
        @JvmField var rpmSmall = 3260
        @JvmField var rpmBig = 2775
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))
        val timeKeep = TimeKeep()

        robot.limelight.setPipeline(1)

        val intakePosition = ButtonReader { gamepad2.y}
        val shootGreen = ButtonReader { gamepad2.a}
        val shootPurple = ButtonReader { gamepad2.b}
        val shootAll = ButtonReader { gamepad2.x}
        val fingerUp = ButtonReader {gamepad2.dpad_up}
        val fingerDown = ButtonReader {gamepad2.dpad_down}
        val highRpm = ButtonReader {gamepad2.right_bumper}
        val lowRpm = ButtonReader {gamepad2.left_bumper}
        val stopShooter = ButtonReader {gamepad2.dpad_left}
        val snipe = ToggleButtonReader({gamepad1.x})
        val autoRpm = ButtonReader {gamepad2.dpad_right}
        val buttons = listOf(intakePosition, shootGreen, shootPurple, shootAll, fingerUp, fingerDown, highRpm, lowRpm, stopShooter, snipe, autoRpm)
        val stopButton = ButtonReader {gamepad2.touchpad}

        robot.transfer.finger.position = 0.9

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            buttons.forEach { it.readValue() }
            robot.drive.updatePoseEstimateOdo()

            robot.transfer.updateBallSlot()

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

            if (fingerUp.wasJustPressed())
                robot.transfer.fingerUp()
            if (fingerDown.wasJustPressed())
                robot.transfer.fingerDown()


            var shootAllActive = false
            if (shootAll.wasJustPressed()) {
                shootAllActive = true
            }


            /// Intake

            if (gamepad1.right_bumper) {
                robot.intake.power = 0.8
            }
            else if (gamepad1.left_bumper) {
                robot.intake.power = -1.0
            }
            else {
                robot.intake.power = 0.0
            }

            var rpm = 0.0
            if (highRpm.wasJustPressed()){ /// shoot far
                robot.shooter.goToRmp(BlindDrive.rpmSmall.toDouble())
            }
            else if (lowRpm.wasJustPressed()) { /// shoot close
                robot.shooter.goToRmp(BlindDrive.rpmBig.toDouble())
            }
            else if (autoRpm.wasJustPressed()) {
                rpm = robot.limelight.getRpm()
                robot.shooter.goToRmp(rpm)
            }
            else if (stopShooter.wasJustPressed()) { /// stop shoot
                robot.shooter.goToRmp(0.0)
            }

            robot.shooter.updateRpm(timeKeep.deltaTime)
            robot.limelight.updateHeadingError()

            telemetry.addData("distance", robot.limelight.getDistance())
            telemetry.addData("auto rpm", rpm)

            //telemetry.addData("target heading", robot.limelight.)
            telemetry.addData("error heading", robot.limelight.headingErrorDeg)
            telemetry.addData("a was pressed (set)", gamepad1.a)
            telemetry.addData("x was pressed (left)", gamepad1.x)
            telemetry.addData("b was pressed (right)", gamepad1.b)
            telemetry.addData("up was pressed (set)", gamepad1.dpad_up)
            telemetry.addData("down was pressed (left)", gamepad1.dpad_down)
            telemetry.addData("rpm", robot.shooter.rpm)
            telemetry.addData("target rpm", robot.shooter.targetRpm)
            telemetry.addData("fingir pos", robot.transfer.finger.position)
            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.update()
        }
    }
}