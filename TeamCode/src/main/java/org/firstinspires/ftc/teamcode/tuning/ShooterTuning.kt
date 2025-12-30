package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.commonlibs.units.rpm
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.robot.Shooter

@TeleOp(group = "tuning")
class ShooterTuning : LinearOpMode(){
    data object ShooterTuningConfig {
        @JvmField
        var kP = Shooter.ShooterConfig.pidController.kP
        @JvmField
        var kI = Shooter.ShooterConfig.pidController.kI
        @JvmField
        var kD = Shooter.ShooterConfig.pidController.kD
        @JvmField
        var kV = Shooter.ShooterConfig.kV
        @JvmField
        var kS = Shooter.ShooterConfig.kS
        @JvmField
        var targetRPM = 0.0
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val shooter = Shooter(hardwareMap)
        val timeKeep = TimeKeep()

        waitForStart()

        while (opModeIsActive()) {
            Shooter.ShooterConfig.apply {
                pidController.apply {
                    kP = ShooterTuningConfig.kP
                    kI = ShooterTuningConfig.kI
                    kD = ShooterTuningConfig.kD
                }
                kV = ShooterTuningConfig.kV
                kS = ShooterTuningConfig.kS
            }

            shooter.targetRpm = ShooterTuningConfig.targetRPM.rpm

            timeKeep.resetDeltaTime()
            shooter.update(timeKeep.deltaTime)
            shooter.addTelemetry(telemetry)
            telemetry.update()
        }
    }
}