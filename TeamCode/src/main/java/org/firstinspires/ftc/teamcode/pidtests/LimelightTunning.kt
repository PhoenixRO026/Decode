package org.firstinspires.ftc.teamcode.pidtests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.commonlibs.units.Pose
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.library.controller.LowPassFilter
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import org.firstinspires.ftc.teamcode.robot.Robot

@TeleOp
class LimelightTunning : LinearOpMode() {
    @Config
    data object LimeLightConfig {
        @JvmField
        var controller = PIDController(
            0.015,
            0.0,
            0.001,
            0.0,
            newTargetReset = true,
            zeroTargetReset = true,
            derivativeFilter = LowPassFilter(0.0),
            stabilityThreshold = 0.0
        )
        @JvmField
        var targetHeading = 0.0
        @JvmField
        var maxTurnPower = 0.5
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val robot = Robot(hardwareMap, Pose(0.0.cm, 0.0.cm, 0.0.deg))

        waitForStart()
        val timeKeep = TimeKeep()

        robot.limelight.setPipeline(1)

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()

            robot.limelight.updateHeadingError()

            if (robot.limelight.tagVisible) {
                val headingErrorDeg = robot.limelight.headingErrorDeg

                var newHeading = LimeLightConfig.controller.calculate(
                    0.0,
                    headingErrorDeg,
                    timeKeep.deltaTime
                )

                newHeading = newHeading.coerceIn(-LimeLightConfig.maxTurnPower, LimeLightConfig.maxTurnPower)

                robot.drive.driveFieldCentric(0.0, 0.0, -newHeading)

                telemetry.addData("headingError", headingErrorDeg)
                telemetry.addData("pidOutput", newHeading)
            }
            telemetry.addData("tagVisible", robot.limelight.tagVisible)
            telemetry.update()
        }
    }
}
