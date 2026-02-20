package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.psikit.LoggedOpMode
import org.firstinspires.ftc.teamcode.psikit.MockTelemetry
import org.firstinspires.ftc.teamcode.psikit.PsiKitOpMode
import org.firstinspires.ftc.teamcode.robot.Drive
import org.psilynx.psikit.core.Logger
import org.psilynx.psikit.core.rlog.RLOGServer
import org.psilynx.psikit.core.rlog.RLOGWriter
import org.psilynx.psikit.ftc.DriverStationLogger
import org.psilynx.psikit.ftc.FtcLoggingSession
import org.psilynx.psikit.ftc.PinpointOdometryLogger

@TeleOp
class TestPsiKit: LoggedOpMode() {
    lateinit var drive: Drive

    override fun runLoggedOpMode() {
        drive = Drive(hardwareMap)

        waitForStart()

        while (isActive()) {
            drive.updatePoseEstimate()

            /// Drive
            drive.isSlowMode = gamepad1.right_trigger >= 0.2
            drive.driveFieldCentric(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble()
            )
            if (gamepad1.y) {
                drive.resetFieldCentric()
            }

            telemetry.addData("slowMode", drive.isSlowMode)
            telemetry.update()
        }
    }
}