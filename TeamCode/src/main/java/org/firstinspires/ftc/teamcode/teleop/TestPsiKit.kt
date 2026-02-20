package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LoggedOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robot.Drive

@TeleOp
class TestPsiKit: LoggedOpMode() {
    lateinit var drive: Drive

    override fun runOpMode() {
        drive = Drive(hardwareMap)

        waitForStart()

        while (opModeIsActive()) {
            drive.updatePoseEstimate()

            /// Drive
            drive.isSlowMode = gamepad1.right_trigger >= 0.2
            drive.driveFieldCentric(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble()
            )
            if (gamepad1.y) {
                drive.resetFieldCentric()
            }

            telemetry.addData("slowMode", drive.isSlowMode)
            telemetry.update()
        }
    }
}