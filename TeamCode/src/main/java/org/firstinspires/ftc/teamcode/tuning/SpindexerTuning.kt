package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.robot.Shooter
import org.firstinspires.ftc.teamcode.robot.Spindexer

@TeleOp(group = "tuning")
class SpindexerTuning : LinearOpMode(){
    data object SpindexerTuningConfig {
        //ATTENTION!: put the tuned values in the spindexer class, not here!!!
        @JvmField
        var kP = Spindexer.SpindexerConfig.pidController.kP
        @JvmField
        var kI = Spindexer.SpindexerConfig.pidController.kI
        @JvmField
        var kD = Spindexer.SpindexerConfig.pidController.kD
        @JvmField
        var targetTicks = 0
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val spindexer = Spindexer(hardwareMap)
        val timeKeep = TimeKeep()

        waitForStart()

        while (opModeIsActive()) {
            Spindexer.SpindexerConfig.pidController.apply {
                kP = SpindexerTuningConfig.kP
                kI = SpindexerTuningConfig.kI
                kD = SpindexerTuningConfig.kD
            }

            spindexer.targetTicks = SpindexerTuningConfig.targetTicks

            timeKeep.resetDeltaTime()
            spindexer.update(timeKeep.deltaTime)
            spindexer.addTelemetry(telemetry)
            telemetry.update()
        }
    }
}