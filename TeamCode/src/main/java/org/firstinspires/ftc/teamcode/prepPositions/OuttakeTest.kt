package org.firstinspires.ftc.teamcode.teleop.prepPositions

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.config.Config
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
import kotlin.math.max
import kotlin.math.min

@TeleOp
class OuttakeTest : LinearOpMode() {
    @Config
    data object outtakeConfig {
        @JvmField var sampleWindow = 0.1
        @JvmField var TICKS_PER_REV = 8192.0
        @JvmField var targetRPM = 3400
    }

    override fun runOpMode() {

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val robot = Robot(hardwareMap)

        var lastTime = now()
        var lastResetTime = now()
        var rpm = 0.0
        val timeKeep = TimeKeep()

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            val currentTime = now()
            val dt = currentTime - lastTime

            if (currentTime - lastResetTime >= outtakeConfig.sampleWindow) {
                val pos = robot.shooter.encoder.getPositionAndVelocity().position
                val elapsed = currentTime - lastResetTime
                val revs = pos / outtakeConfig.TICKS_PER_REV
                rpm = (revs / elapsed) * 60.0


                robot.drive.mecanumDrive.rightBack.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                robot.drive.mecanumDrive.rightBack.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

                lastResetTime = currentTime
            }

            robot.shooter.outtakeTargetRpm = outtakeConfig.targetRPM

            robot.shooter.update(timeKeep.deltaTime)

            telemetry.addData("RPM", "%.2f", rpm)
            telemetry.addData("Target RPM", outtakeConfig.targetRPM)
            telemetry.addData("Power", "%.3f", robot.shooter.power)
            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.update()

            lastTime = currentTime
        }
    }
}