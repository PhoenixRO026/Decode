package org.firstinspires.ftc.teamcode.prepPositions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.commonlibs.units.Pose
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.library.buttons.ButtonReader
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.Spindexer

@TeleOp
class TransferPrep : LinearOpMode() {
    var action : Action? = null

    override fun runOpMode() {
        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))
        val timeKeep = TimeKeep()

        val intakeBalls = ButtonReader { gamepad1.y }
        val shootBalls = ButtonReader { gamepad1.a }

        val buttons = listOf(intakeBalls, shootBalls)

        waitForStart()

        robot.transfer.goToPos(Spindexer.TransferPos.intake0)

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            buttons.forEach { it.readValue() }

            if(intakeBalls.wasJustPressed()) {
                action = robot.intakeBalls(Spindexer.TransferPos.shoot0)
            }
            else if(shootBalls.wasJustPressed() && action == null) {
                action = robot.shootBalls()
            }
            else if(gamepad1.b) {
                robot.transfer.goToPos(Spindexer.TransferPos.intake0)
            }
            telemetry.addData("color", robot.transfer.sensorColor)
            telemetry.addData("slot 0", robot.transfer.slots[0])
            telemetry.addData("slot 1", robot.transfer.slots[1])
            telemetry.addData("slot 2", robot.transfer.slots[2])
            telemetry.addData("target pos", robot.transfer.currentPos)
            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.update()
            runActions()
        }
    }
    private fun runActions() {
        action?.let {
            if (!it.run(TelemetryPacket())) {
                action = null
            }
        }
    }
}