package org.firstinspires.ftc.teamcode.prepPositions

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.acmerobotics.roadrunner.now
import com.commonlibs.units.Pose
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.commonlibs.units.pose
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.library.buttons.ToggleButtonReader
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import org.firstinspires.ftc.teamcode.robot.Intake
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.teleop.prepPositions.OuttakeTest.outtakeConfig

@TeleOp
class SystemsTest: LinearOpMode() {

    @Config
    data object teleConfig {
        @JvmField
        var controller = PIDController(
            kP = 0.0015,
            kI = 0.000001,
            kD = 0.0045,
            stabilityThreshold = 0.2
        )
    }
    private var driver1Action: Action? = null
    private val timeKeep = TimeKeep()

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.addLine("INITIALIZING")
        telemetry.update()

        timeKeep.resetDeltaTime()
        val robot = Robot(
            hardwareMap = hardwareMap,
            pose = Pose(0.0.cm, 0.0.cm, 0.0.deg),
            resetEncoders = false
        )

        val intake = ToggleButtonReader({gamepad1.x})
        val outtake = ToggleButtonReader({gamepad1.b})
        val moveRight = ToggleButtonReader({gamepad2.b})
        val moveStart = ToggleButtonReader({gamepad2.x})
        val buttons = listOf(intake,outtake,moveRight,moveStart)

        var curr = 170

        /*enum class SensorColor{
            GREEN,
            PURPLE,
            NONE
        }
        var sensorHue: Float = 0f
        var hsv = floatArrayOf(0f,0f,0f)

        val sensorColor get() = when{
            hsv[1] != 0f && sensorHue in 60f .. 180f->SensorColor.GREEN
            else SensorColor.NONE
        }*/


        waitForStart()

        while (opModeIsActive()){
            buttons.forEach { it.readValue() }


            if (intake.state){
                robot.intake.motor.power = 1.0
            }
            else{
                robot.intake.power = 0.0
            }
            if (outtake.state){
                robot.shooter.power = 1.0
            }
            else{
                robot.shooter.power = 0.0
            }

            /*if (outtake.state){
                motor2.power=1.0
            }
            else{
                motor1.power=0.0
            }*/

            if (gamepad1.a)
                robot.transfer.finger.position = 0.6494


            if (gamepad1.y)
                robot.transfer.finger.position = 0.951

            if (moveRight.wasJustPressed()) {
                curr = curr + 179
                driver1Action = robot.transfer.goToPosAction(curr.toDouble())
            }
            if (moveStart.wasJustPressed()){
                driver1Action = robot.transfer.goToPosAction(curr.toDouble())
            }


            //motorTransfer.power = gamepad1.right_stick_y.toDouble() / 2

        }
    }
}