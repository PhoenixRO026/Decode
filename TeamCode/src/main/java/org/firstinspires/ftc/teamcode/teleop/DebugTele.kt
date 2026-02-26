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
import org.firstinspires.ftc.teamcode.robot.Spindexer


@TeleOp
open class DebugTele : LinearOpMode(){
    open val pipeline: Int = 1
    @Config
    data object BlindDrive {
        @JvmField var rpmSmall = 3300
        @JvmField var rpmBig = 2800
    }
    private var driver1Action: Action? = null

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))
        val timeKeep = TimeKeep()

        robot.limelight.setPipeline(pipeline)

        val shootGreen = ButtonReader { gamepad2.x }
        val shootPurple = ButtonReader { gamepad2.b }
        val shootAll = ButtonReader { gamepad2.y }
        val highRpm = ButtonReader {gamepad2.right_bumper}
        val lowRpm = ButtonReader {gamepad2.left_bumper}
        val stopShooter = ButtonReader {gamepad2.dpad_left}
        val intakeBalls = ToggleButtonReader ({gamepad1.x})
        val buttons = listOf(shootGreen, shootPurple, shootAll, highRpm, lowRpm, stopShooter, intakeBalls)

        waitForStart()

        robot.transfer.fingerDown()
        robot.transfer.servoTransfer1.position = 0.0400
        robot.transfer.servoTransfer2.position = 0.0400

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            buttons.forEach { it.readValue() }
            robot.drive.updatePoseEstimateOdo()

            /// Drive

            if (gamepad1.left_trigger >= 0.2) {
                robot.drive.isSlowMode = true
            } else {
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

            if(gamepad1.dpad_left) {
                driver1Action = robot.intakeBalls(Spindexer.TransferPos.shoot0)
            }
            if(gamepad1.dpad_down) {
                driver1Action = robot.intakeBalls(Spindexer.TransferPos.shoot1)
            }
            if(gamepad1.dpad_right) {
                driver1Action = robot.intakeBalls(Spindexer.TransferPos.shoot2)
            }
            if (gamepad1.a) {
                driver1Action = robot.shootBalls(robot.shooter.rpmFar)
            }
            if (gamepad1.x) {
                robot.shooter.goToRmp(robot.shooter.rpmFar)
            }
            if (gamepad1.b) {
                robot.shooter.goToRmp(0.0)
            }
            robot.shooter.updateRpm(timeKeep.deltaTime)

            telemetry.addData("transfer pos", when (robot.transfer.currentPos) {
                Spindexer.TransferPos.intake0 -> "intake0"
                Spindexer.TransferPos.intake1 -> "intake1"
                Spindexer.TransferPos.intake2 -> "intake2"
                Spindexer.TransferPos.shoot1 -> "shoot1"
                Spindexer.TransferPos.shoot2 -> "shoot2"
                Spindexer.TransferPos.shoot0 -> "shoot0"
                Spindexer.TransferPos.pseudo0 -> "pseudo0"
                Spindexer.TransferPos.pseudo2 -> "pseudo2"
            })
            telemetry.addData("slot 0", robot.transfer.slots[0])
            telemetry.addData("slot 1", robot.transfer.slots[1])
            telemetry.addData("slot 2", robot.transfer.slots[2])

            telemetry.addData("distance", robot.limelight.getDistance())
            telemetry.addData("auto rpm", robot.shooter.rpm)

            telemetry.addData("error heading", robot.limelight.headingErrorDeg)
            telemetry.addData("turret power", robot.shooter.powerTurret)
            telemetry.addData("target pos", robot.shooter.targetPos)


            runActions()

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