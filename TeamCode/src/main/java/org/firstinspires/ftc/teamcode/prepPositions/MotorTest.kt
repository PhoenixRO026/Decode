package org.firstinspires.ftc.teamcode.teleop.prepPositions

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import org.firstinspires.ftc.teamcode.teleop.prepPositions.OuttakeTest.outtakeConfig

@TeleOp
class MotorTest : LinearOpMode() {

    @Config
    data object motorConfig {
        @JvmField var basePower1 = 0.0
        @JvmField var basePower2 = 0.0

        @JvmField var sampleWindow = 0.1
        @JvmField var TICKS_PER_REV = 8192.0

        @JvmField var targetRPM = 3400.0

        var controller = PIDController(
            kP = 0.0006,
            kI = 0.0001,
            kD = 0.0000,
            stabilityThreshold = 100.0
        )

        @JvmField var integralLimit = 5000.0

        // optional feedforward (power bias)
        @JvmField var kF = 0.0

        // filtering
        @JvmField var rpmFilterGain = 0.3
    }

    override fun runOpMode() {
        val motorShooterTop = hardwareMap.get(DcMotorEx::class.java, "motorShooterTop")
        val motorShooterBottom = hardwareMap.get(DcMotorEx::class.java, "motorShooterBottom")

        motorShooterTop.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorShooterTop.direction = DcMotorSimple.Direction.REVERSE
        motorShooterTop.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        motorShooterBottom.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorShooterBottom.direction = DcMotorSimple.Direction.FORWARD
        motorShooterBottom.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        val shooterEncoder = RawEncoder(motorShooterBottom)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        var lastTime = now()
        var lastResetTime = now()
        var rpm = 0.0//test
        val timeKeep = TimeKeep()

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            val currentTime = now()
            val dt = currentTime - lastTime

            if (currentTime - lastResetTime >= outtakeConfig.sampleWindow) {
                val pos = motorShooterBottom.currentPosition
                val elapsed = currentTime - lastResetTime
                val revs = pos / outtakeConfig.TICKS_PER_REV
                rpm = (revs / elapsed) * 60.0

                motorShooterBottom.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                motorShooterBottom.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                lastResetTime = currentTime
            }

            motorShooterTop.power= motorConfig.basePower1
            motorShooterBottom.power= motorConfig.basePower1

            telemetry.addData("Motor1 Power", motorShooterTop.power)
            telemetry.addData("Motor2 Power", motorShooterBottom.power)
            telemetry.addData("rpm", rpm)
            telemetry.update()
        }
    }
}
