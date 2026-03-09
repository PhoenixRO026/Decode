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
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.Shooter.ShooterConfig
import org.firstinspires.ftc.teamcode.teleop.tests.TurretPosTunning.PositionTunningConfic.ticksPerRev

@TeleOp
class TurretPosTunning : LinearOpMode() {
    @Config
    data object PositionTunningConfic {
        @JvmField
        var controller = PIDController(
            kP = 0.001,
            kD = 0.000027,
            kI = 0.00125,
            stabilityThreshold = 0.2
        )
        @JvmField
        var targetPos = 0.0
        @JvmField
        var ticksPerRev = 8192.0 * (108.0/ 22.0)
        var DegPerTick = 360 / ticksPerRev
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val robot = Robot(hardwareMap)

        val timeKeep = TimeKeep()

        fun tickToDeg(ticks : Double) : Double {
            return (360 / ticksPerRev) * ticks
        }

        fun degToTick(deg : Double) : Double {
            return (ShooterConfig.ticksPerRev / 360.0) * deg
        }

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()

            robot.shooter.goToPos(PositionTunningConfic.targetPos)
            robot.shooter.updateTurretPosition(timeKeep.deltaTime)

            telemetry.addData("transfer target pos", robot.shooter.targetPos)
            telemetry.addData("transfer pos", robot.shooter.turretPosition)

            telemetry.addData("pos in deg", tickToDeg(robot.shooter.turretPosition))/// 22 -> 15 -> 108 15/22
            telemetry.addData("target in deg", tickToDeg(robot.shooter.targetPos))

            telemetry.addData("transfer power", robot.shooter.powerTurret)

            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.update()

        }
    }
}