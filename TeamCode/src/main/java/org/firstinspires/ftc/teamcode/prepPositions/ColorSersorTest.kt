package org.firstinspires.ftc.teamcode.prepPositions

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.Spindexer

@TeleOp
class ColorSersorTest : LinearOpMode() {
    override fun runOpMode() {
        val servoTransferFront = hardwareMap.get(Servo::class.java, "servoTransferFront")
        val servoTransferBack = hardwareMap.get(Servo::class.java, "servoTransferBack")
        val finger = hardwareMap.get(Servo::class.java, "finger")

        val colorSensor = hardwareMap.get(NormalizedColorSensor::class.java, "colorSensor")

        var gain = 1f

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val robot = Robot(hardwareMap)
        val timeKeep = TimeKeep()

        waitForStart()

        while (opModeIsActive()) {


            // Update the gain value if either of the A or B gamepad buttons is being held
            if (gamepad1.a) {
                // Only increase the gain by a small amount, since this loop will occur multiple times per second.
                gain += 0.005.toFloat()
            } else if (gamepad1.y && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
                gain -= 0.005.toFloat()
            }

            colorSensor.setGain(gain)

            robot.transfer.updateHue()

            if(gamepad1.x) {
                robot.transfer.goToNextIntake()
            }
            if (gamepad1.b) {
                robot.transfer.goToPos(Spindexer.TransferPos.intake0)
            }

            telemetry.addData("distance",
                (robot.transfer.colorSensor as DistanceSensor).getDistance(DistanceUnit.MM))
            telemetry.addData("Gain", gain)
            telemetry.addData("color", robot.transfer.sensorColor)
            telemetry.addData("hue", robot.transfer.sensorHue)
            telemetry.addData("hsv", robot.transfer.hsv)
            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.update()

        }
    }
}