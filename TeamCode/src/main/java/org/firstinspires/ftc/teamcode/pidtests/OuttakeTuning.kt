package org.firstinspires.ftc.teamcode.pidtests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.acmerobotics.roadrunner.now
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import kotlin.math.abs

@TeleOp
class OuttakeTuning : LinearOpMode() {
    @Config
    data object outtakeConfig {
        @JvmField var sampleWindow = 0.1
        @JvmField var TICKS_PER_REV = 8192.0
        @JvmField var targetRPM = 3400
    }
    @Config
    data object OuttakeTuningConfig {
        @JvmField
        var controller = PIDController(
            kP = 0.001995,
            kD = 0.0000001,
            kI = 0.0000001,
            stabilityThreshold = 0.2
        )
        @JvmField
        var targetRpm = 0.0
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val motorShooterTop = hardwareMap.get(DcMotorEx::class.java, "motorShooterTop")
        val motorShooterBottom = hardwareMap.get(DcMotorEx::class.java, "motorShooterBottom")

        motorShooterTop.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorShooterTop.direction = DcMotorSimple.Direction.FORWARD
        motorShooterTop.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        motorShooterBottom.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorShooterBottom.direction = DcMotorSimple.Direction.REVERSE
        motorShooterBottom.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val shooterEncoder = RawEncoder(motorShooterBottom)


        var lastTime = now()
        var lastResetTime = now()
        val timeKeep = TimeKeep()
        var rpm = 0.0
        var targetRpm = 0.0
        var shooterPower = 0.0

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            val currentTime = now()
            val dt = currentTime - lastTime

            if (currentTime - lastResetTime >= outtakeConfig.sampleWindow) {
                val pos = shooterEncoder.getPositionAndVelocity().position
                val elapsed = currentTime - lastResetTime
                val revs = pos / outtakeConfig.TICKS_PER_REV
                rpm = abs((revs / elapsed) * 60.0)

                motorShooterBottom.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                motorShooterBottom.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

                lastResetTime = currentTime
            }

            targetRpm = OuttakeTuningConfig.targetRpm

            shooterPower = OuttakeTuningConfig.controller.calculate(abs(rpm),abs(targetRpm),timeKeep.deltaTime)
            /// how to target toleramce???
            motorShooterBottom.power = shooterPower
            motorShooterTop.power = shooterPower

            telemetry.addData("RPM", "%.2f", rpm)
            telemetry.addData("Target RPM", OuttakeTuningConfig.targetRpm)
            telemetry.addData("Power", "%.3f", motorShooterBottom.power)
            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.update()

            lastTime = currentTime
        }
    }
}