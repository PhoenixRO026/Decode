package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.now
import com.commonlibs.units.Pose
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.teleop.prepPositions.OuttakeTest

@TeleOp
class CanonEventTele : LinearOpMode(){
    @Config
    data object teleConfig {
        @JvmField var sampleWindow = 0.1
        @JvmField var TICKS_PER_REV = 8192.0
        @JvmField var targetRPM = 3400

        @JvmField var intakePower = 0.0
        @JvmField var targetPos = 0.0
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))

        var lastTime = now()
        var lastResetTime = now()
        var rpm = 0.0
        val timeKeep = TimeKeep()

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            val currentTime = now()
            val dt = currentTime - lastTime

            if (currentTime - lastResetTime >= teleConfig.sampleWindow) {
                val pos = robot.shooter.encoder.getPositionAndVelocity().position
                val elapsed = currentTime - lastResetTime
                val revs = pos / teleConfig.TICKS_PER_REV
                rpm = (revs / elapsed) * 60.0

                robot.drive.mecanumDrive.rightBack.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                robot.drive.mecanumDrive.rightBack.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                lastResetTime = currentTime
            }

            robot.shooter.outtakeTargetRpm = teleConfig.targetRPM

            robot.intake.power = teleConfig.intakePower
            robot.transfer.targetPosition = teleConfig.targetPos

            robot.shooter.update(timeKeep.deltaTime)
            robot.transfer.update(timeKeep.deltaTime)

            telemetry.addData("rpm", "%.2f", rpm)
            telemetry.addData("target RPM", OuttakeTest.outtakeConfig.targetRPM)
            telemetry.addData("power shooter", "%.3f", robot.shooter.power)
            telemetry.addData("pos", robot.transfer.position)
            telemetry.addData("target pos", robot.transfer.targetPosition)
            telemetry.addData("power trans", robot.transfer.power)
            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.update()

            lastTime = currentTime
            /*
            robot.drive.isSlowMode = gamepad1.right_trigger >= 0.2
            robot.drive.driveFieldCentric(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble()
            )
            if (gamepad1.y) {
                robot.drive.resetFieldCentric()
            }*/

        }
    }
}