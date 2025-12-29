package org.firstinspires.ftc.teamcode.teleop.prepPositions

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.config.Config
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.robot_old.Robot

@TeleOp
class SensorTest : LinearOpMode() {
    @Config
    data object sensorConfig{
        @JvmField
        var motorPower = 0.0
        @JvmField
        var sensorGain = 1.0f

    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val timeKeep = TimeKeep()
        val intake = Robot(hardwareMap).intake

        waitForStart()



        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            intake.motor.power = sensorConfig.motorPower

            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.update()
        }
    }
}
