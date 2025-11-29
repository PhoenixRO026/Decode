package org.firstinspires.ftc.teamcode.teleop.prepPositions

import android.graphics.Color
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.now
import com.commonlibs.units.cm
import com.commonlibs.units.s
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import org.firstinspires.ftc.teamcode.library.controller.LowPassFilter
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.teleop.prepPositions.OuttakeTest.outtakeConfig
import kotlin.math.abs
import kotlin.math.sign

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
