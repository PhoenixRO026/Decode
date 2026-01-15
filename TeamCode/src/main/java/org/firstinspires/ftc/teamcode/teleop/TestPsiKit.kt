package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.psikit.MockTelemetry
import org.firstinspires.ftc.teamcode.robot.Drive
import org.psilynx.psikit.core.Logger
import org.psilynx.psikit.core.rlog.RLOGServer
import org.psilynx.psikit.core.rlog.RLOGWriter
import org.psilynx.psikit.ftc.DriverStationLogger
import org.psilynx.psikit.ftc.FtcLoggingSession
import org.psilynx.psikit.ftc.PinpointOdometryLogger
import org.psilynx.psikit.ftc.PsiKitOpMode

@TeleOp
class TestPsiKit: PsiKitOpMode() {
    lateinit var drive: Drive
    val driverStationLogger = DriverStationLogger()
    val pinpointOdometryLogger = PinpointOdometryLogger()

    override fun psiKit_init() {
        telemetry = MockTelemetry(telemetry)

        val server = RLOGServer()
        val writer = RLOGWriter("log.rlog")

        Logger.addDataReceiver(server)
        Logger.addDataReceiver(writer)

        drive = Drive(hardwareMap)

        processHardwareInputs()
    }

    override fun psiKit_init_loop() {
        processHardwareInputs()
    }

    override fun psiKit_start() {

    }

    override fun psiKit_loop() {
        driverStationLogger.log(gamepad1, gamepad2)
        pinpointOdometryLogger.logAll(hardwareMap)

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

    override fun psiKit_stop() {

    }
}