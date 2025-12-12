package org.firstinspires.ftc.teamcode.teleop.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.ftc.Encoder
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
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.Shooter
import org.firstinspires.ftc.teamcode.teleop.prepPositions.OuttakeTest.outtakeConfig
import kotlin.math.abs
import kotlin.time.toDuration

@TeleOp
class TransferTuning : LinearOpMode() {
    @Config
    data object TransferTuningConfig {
        @JvmField
        var controller = PIDController(
            kP = 0.0015,
            kD = 0.000001,
            kI = 0.0045,
            stabilityThreshold = 50.0
        )
        @JvmField
        var kV = 0.00025
        @JvmField
        var targetPos = 0.0
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val motorTransfer = hardwareMap.get(DcMotorEx::class.java, "motorTransfer")

        motorTransfer.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorTransfer.direction = DcMotorSimple.Direction.FORWARD
        motorTransfer.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val encoderTransfer : Encoder = RawEncoder(motorTransfer)

        val timeKeep = TimeKeep()
        var transferPower = 0.0
        var targetPos = 0.0
        var position = 0.0

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()

            position = encoderTransfer.getPositionAndVelocity().position
            targetPos = TransferTuningConfig.targetPos
            motorTransfer.power = TransferTuningConfig.controller.calculate(position, targetPos, timeKeep.deltaTime)

            telemetry.addData("transfer target pos", TransferTuningConfig.targetPos)
            telemetry.addData("transfer pos", position)
            telemetry.addData("transfer power", motorTransfer.power)

            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.update()

        }
    }
}