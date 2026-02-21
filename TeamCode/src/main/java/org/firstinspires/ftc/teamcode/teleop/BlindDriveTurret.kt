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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.library.buttons.ButtonReader
import org.firstinspires.ftc.teamcode.library.buttons.ToggleButtonReader
import org.firstinspires.ftc.teamcode.robot.Robot


@TeleOp
abstract class BlindDriveTurret : LinearOpMode(){
    abstract val pipeline : Int
    @Config
    data object BlindDrive {
        @JvmField var rpmSmall = 3260
        @JvmField var rpmBig = 2775
    }

    private var driver1Action: Action? = null

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))
        val timeKeep = TimeKeep()

        robot.limelight.setPipeline(pipeline)

        val shootGreen = ButtonReader { gamepad2.a}
        val shootPurple = ButtonReader { gamepad2.b}
        val shootAll = ButtonReader { gamepad2.x}
        val fingerUp = ButtonReader {gamepad2.dpad_up}
        val fingerDown = ButtonReader {gamepad2.dpad_down}
        val highRpm = ButtonReader {gamepad2.right_bumper}
        val lowRpm = ButtonReader {gamepad2.left_bumper}
        val stopShooter = ButtonReader {gamepad2.dpad_left}
        val snipe = ToggleButtonReader ({gamepad1.x})
        val nextIntake = ButtonReader {gamepad2.y}
        val nextShoot = ButtonReader {gamepad2.x}
        val buttons = listOf(shootGreen, shootPurple, shootAll, fingerUp, fingerDown, highRpm, lowRpm, stopShooter, snipe, nextIntake, nextShoot, nextIntake)


        robot.transfer.finger.position = 0.9

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            buttons.forEach { it.readValue() }
            robot.drive.updatePoseEstimateOdo()

            /// Drive

            if(gamepad1.left_trigger >= 0.2) {
                robot.drive.isSlowMode = true
            }
            else {
                robot.drive.isSlowMode = false
            }

            robot.drive.driveFieldCentric(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble()
            )
            if (gamepad1.y) {
                robot.drive.resetFieldCentric()
            }

            /// Intake
            if (snipe.state) {
                if (driver1Action == null) {
                    driver1Action = SequentialAction(
                        robot.intakeTeleBalls(),
                        InstantAction { snipe.setState(false) }
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
                if (nextIntake.wasJustPressed()) {
                    robot.transfer.goToNextIntake()
                }

                if (nextShoot.wasJustPressed()) {
                    robot.transfer.goToNextShoot()
                }
            }

            /// Transfer

            if (fingerUp.wasJustPressed())
                robot.transfer.fingerUp()
            if (fingerDown.wasJustPressed())
                robot.transfer.fingerDown()

            if (shootGreen.wasJustPressed()) {
                robot.shootGreen()
            }
            if (shootPurple.wasJustPressed()) {
                robot.shootPurple()
            }

            if (highRpm.wasJustPressed()){ /// shoot far
                robot.shooter.goToRmp(robot.shooter.rpmFar)
            }
            else if (lowRpm.wasJustPressed()) { /// shoot close
                robot.shooter.goToRmp(robot.shooter.rpmClose)
            }
            else if (stopShooter.wasJustPressed()) { /// stop shoot
                robot.shooter.goToRmp(0.0)
            }

            robot.shooter.updateRpm(timeKeep.deltaTime)
            robot.limelight.updateHeadingError()
            robot.shooter.updateTurretTargetPos(timeKeep.deltaTime, robot.limelight.headingErrorDeg)

            robot.shooter.addTelemetry(telemetry)

            telemetry.addData("distance", robot.limelight.getDistance())
            telemetry.addData("auto rpm", robot.shooter.rpm)

            telemetry.addData("error heading", robot.limelight.headingErrorDeg)
            telemetry.addData("turret power", robot.shooter.powerTurret)
            telemetry.addData("target pos", robot.shooter.targetPos)

//            telemetry.addData("a was pressed (set)", gamepad1.a)
//            telemetry.addData("x was pressed (left)", gamepad1.x)
//            telemetry.addData("b was pressed (right)", gamepad1.b)
//            telemetry.addData("up was pressed (set)", gamepad1.dpad_up)
//            telemetry.addData("down was pressed (left)", gamepad1.dpad_down)
//            telemetry.addData("rpm", robot.shooter.rpm)
//            telemetry.addData("target rpm", robot.shooter.targetRpm)
//            telemetry.addData("fingir pos", robot.transfer.finger.position)
//            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
//            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.update()
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