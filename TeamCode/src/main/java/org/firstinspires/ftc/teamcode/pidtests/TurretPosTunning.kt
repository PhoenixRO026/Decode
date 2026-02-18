package org.firstinspires.ftc.teamcode.teleop.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.ftc.Encoder
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
class TurretPosTunning : LinearOpMode() {
    @Config
    data object PositionTunningConfic {
        @JvmField
        var controller = PIDController(
            kP = 0.0005,
            kD = 0.00005,
            kI = 0.00045,
            stabilityThreshold = 50.0
        )
        @JvmField
        var kV = 0.00025
        @JvmField
        var targetPos = 0.0
        @JvmField
        var multiplier = 1.0
        @JvmField
        var offset = 0.0
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val motorTurret = hardwareMap.get(DcMotorEx::class.java, "motorTurret")

        motorTurret.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorTurret.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorTurret.direction = DcMotorSimple.Direction.REVERSE
        motorTurret.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val encoderTransfer : Encoder = RawEncoder(motorTurret)

        val timeKeep = TimeKeep()
        var transferPower = 0.0
        var targetPos = 0.0
        var position = 0.0

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()

            position = encoderTransfer.getPositionAndVelocity().position
            targetPos = PositionTunningConfic.targetPos
            motorTurret.power = -PositionTunningConfic.controller.calculate(position, targetPos, timeKeep.deltaTime)

            telemetry.addData("transfer target pos", PositionTunningConfic.targetPos)
            telemetry.addData("transfer pos", position)
            telemetry.addData("transfer power", motorTurret.power)

            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.update()

        }
    }
}