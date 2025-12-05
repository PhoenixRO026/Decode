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
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.teleop.prepPositions.OuttakeTest

class TeleOp : LinearOpMode() {
    @Config
    data object teleConfig {
        @JvmField var sampleWindow = 0.1
        @JvmField var TICKS_PER_REV = 8192.0
        @JvmField var targetRPM = 3400

        @JvmField var intakePower = 1.0
        @JvmField var targetPos = 0.0
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))

        var lastResetTime = now()
        val timeKeep = TimeKeep()

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            val currentTime = now()

            if (currentTime - lastResetTime >= TeleOp.teleConfig.sampleWindow) {
                robot.drive.mecanumDrive.rightBack.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                robot.drive.mecanumDrive.rightBack.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                lastResetTime = currentTime
            }

            robot.shooter.outtakeTargetRpm = CanonEventTele.teleConfig.targetRPM

            robot.transfer.targetPosition = CanonEventTele.teleConfig.targetPos

            robot.shooter.update(timeKeep.deltaTime)
            robot.transfer.update(timeKeep.deltaTime)

            robot.drive.driveFieldCentric(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble()
            )

            if (gamepad2.a) {
                robot.transfer.open_trap()
            }
            if (gamepad2.b) {
                robot.transfer.close_trap()
            }
            if (gamepad2.dpad_up) {
                robot.intake.power = TeleOp.teleConfig.intakePower
            }
            if (gamepad2.dpad_down) {
                robot.intake.power = 0.0
            }
        }
    }
}