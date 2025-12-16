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
import org.firstinspires.ftc.teamcode.library.buttons.ButtonReader
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.teleop.prepPositions.OuttakeTest

@TeleOp
class CanonEventTesting : LinearOpMode(){
    @Config
    data object CanonEventConfig {
        @JvmField var fingerPos = 1.0
        @JvmField var pos = 0.0
        @JvmField var multiplier = 1.0
        @JvmField var offset = 100.0

        @JvmField var shooterTargetRpm = 0
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))

        var lastTime = now()
        var lastResetTime = now()
        var rpm = 0.0
        val timeKeep = TimeKeep()
        var curr = 170
        val set = ButtonReader { gamepad1.a }
        val right = ButtonReader { gamepad1.b}
        val left = ButtonReader { gamepad1.x}
        val up = ButtonReader {gamepad1.dpad_up}
        val down = ButtonReader {gamepad1.dpad_down}
        val increse = ButtonReader {gamepad1.left_bumper}
        val decrese = ButtonReader {gamepad1.right_bumper}
        val buttons = listOf(set, right, left,up,down)

        robot.transfer.finger.position = 1.0

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            val currentTime = now()
            val dt = currentTime - lastTime
            buttons.forEach { it.readValue() }

            if (up.wasJustPressed())
                robot.transfer.finger.position= 0.7

            if (down.wasJustPressed())
                robot.transfer.finger.position = 1.0

            if (set.wasJustPressed())
                robot.transfer.goToPos(curr.toDouble(), CanonEventConfig.multiplier,CanonEventConfig.offset)

            if (right.wasJustPressed()){
                curr+=170;
                robot.transfer.goToPos(curr.toDouble(), CanonEventConfig.multiplier,CanonEventConfig.offset)
            }

            if (left.wasJustPressed()){
                curr-=170;
                robot.transfer.goToPos(curr.toDouble(), CanonEventConfig.multiplier,CanonEventConfig.offset)
            }

            if (increse.wasJustPressed()){
                robot.shooter.goToRmp(CanonEventConfig.shooterTargetRpm)
            }

            telemetry.addData("curr", curr)
            telemetry.addData("a was pressed (set)", gamepad1.a)
            telemetry.addData("x was pressed (left)", gamepad1.x)
            telemetry.addData("b was pressed (right)", gamepad1.b)
            telemetry.addData("up was pressed (set)", gamepad1.dpad_up)
            telemetry.addData("down was pressed (left)", gamepad1.dpad_down)
            telemetry.addData("pos", robot.transfer.position)
            telemetry.addData("fingir pos", robot.transfer.finger.position)
            telemetry.addData("target pos", robot.transfer.targetPosition)
            telemetry.addData("power trans", robot.transfer.power)
            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.update()

            robot.shooter.update(timeKeep.deltaTime)
            robot.transfer.update(timeKeep.deltaTime)
            lastTime = currentTime

            robot.drive.isSlowMode = gamepad1.right_trigger >= 0.2
            robot.drive.driveFieldCentric(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble()
            )
            if (gamepad1.y) {
                robot.drive.resetFieldCentric()
            }
        }
    }
}