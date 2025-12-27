package org.firstinspires.ftc.teamcode.teleop.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.library.controller.PIDController

@TeleOp
class OuttakeTuning : LinearOpMode() {
    @Config
    data object OuttakeTuningConfig {
        @JvmField
        var controller = PIDController(
            kP = 0.0,
            kD = 0.0,
            kI = 0.0,
            stabilityThreshold = 50.0
        )
        @JvmField
        var targetRpm = 0.0
        @JvmField
        var kS = 0.8
        @JvmField
        var kV = 0.002146
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

        val rightBack = hardwareMap.get(DcMotorEx::class.java, "motorRB")

        val encoderOuttake : Encoder = OverflowEncoder(RawEncoder(rightBack))

        encoderOuttake.direction = DcMotorSimple.Direction.REVERSE

        val voltageSensor = hardwareMap.voltageSensor.iterator().next()

        val timeKeep = TimeKeep()
        fun rpm() = encoderOuttake.getPositionAndVelocity().velocity / 28.0 * 60
        var shooterPower: Double

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()

            val voltage = voltageSensor.voltage

            val pidPower = OuttakeTuningConfig.controller.calculate(rpm(), OuttakeTuningConfig.targetRpm, timeKeep.deltaTime)

            val feedforwardPower = OuttakeTuningConfig.kS + OuttakeTuningConfig.kV * OuttakeTuningConfig.targetRpm

            shooterPower = pidPower + feedforwardPower / voltage

            motorShooterBottom.power = shooterPower
            motorShooterTop.power = shooterPower

            telemetry.addData("RPM", rpm())
            telemetry.addData("Target RPM", OuttakeTuningConfig.targetRpm)
            telemetry.addData("Power", "%.3f", shooterPower)
            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.addData("voltage", voltage)
            telemetry.update()
        }
    }
}