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
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import org.firstinspires.ftc.teamcode.robot.Shooter
import org.firstinspires.ftc.teamcode.robot.Spindexer

@TeleOp
class OuttakeTuning : LinearOpMode() {
    @Config
    data object OuttakeTuningConfig {
        @JvmField
        var controller = PIDController(
            kP = 0.002,
            kD = 0.00004,
            kI = 0.018,
            stabilityThreshold = 50.0
        )
        @JvmField
        var targetRpm = 0.0
        @JvmField
        var kS = 1.4
        @JvmField
        var kV = 0.0029
        @JvmField
        var fingerUp = false
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val motorShooterTop = hardwareMap.get(DcMotorEx::class.java, "motorShooterTop")
        val motorShooterBottom = hardwareMap.get(DcMotorEx::class.java, "motorShooterBottom")

        motorShooterTop.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorShooterTop.direction = DcMotorSimple.Direction.REVERSE
        motorShooterTop.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        motorShooterBottom.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorShooterBottom.direction = DcMotorSimple.Direction.FORWARD
        motorShooterBottom.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val encoderOuttake : Encoder = OverflowEncoder(RawEncoder(motorShooterBottom))

        encoderOuttake.direction = DcMotorSimple.Direction.REVERSE

        val voltageSensor = hardwareMap.voltageSensor.iterator().next()

        val finger = hardwareMap.get(Servo::class.java, "finger")

        val timeKeep = TimeKeep()
        fun rpm() = encoderOuttake.getPositionAndVelocity().velocity / 28.0 * 60 * Shooter.ShooterConfig.gearRatio
        var shooterPower: Double

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()

            if (OuttakeTuningConfig.fingerUp) {
                finger.position = Spindexer.TransferConfig.fingerUpPosition
            } else {
                finger.position = Spindexer.TransferConfig.fingerDownPosition
            }

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