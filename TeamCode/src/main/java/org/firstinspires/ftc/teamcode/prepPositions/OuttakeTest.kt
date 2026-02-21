package org.firstinspires.ftc.teamcode.teleop.prepPositions

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.commonlibs.units.Duration
import com.commonlibs.units.Pose
import com.commonlibs.units.Vector2d
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.commonlibs.units.inch
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.library.buttons.ButtonReader
import org.firstinspires.ftc.teamcode.library.buttons.ToggleButtonReader
import org.firstinspires.ftc.teamcode.robot.Robot

@TeleOp
class OuttakeTest : LinearOpMode() {
    private var driver1Action: Action? = null
    val blueGoal = Vector2d(67.inch, 58.inch)
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))
        val timeKeep = TimeKeep()

        robot.limelight.setPipeline(1)

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

            robot.limelight.updateHeadingError()
            robot.shooter.updateTurretTargetPos(timeKeep.deltaTime, robot.limelight.headingErrorDeg)

            robot.shooter.updateTurret(timeKeep.deltaTime)

            telemetry.addData("error", robot.limelight.headingErrorDeg)
            telemetry.addData("pos", robot.shooter.turretPosition)
            telemetry.addData("error in tick", robot.shooter.degToTick(robot.limelight.headingErrorDeg))
            telemetry.addData("target pos", robot.shooter.targetPos)
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