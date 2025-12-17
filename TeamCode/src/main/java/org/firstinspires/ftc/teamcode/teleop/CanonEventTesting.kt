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
import org.firstinspires.ftc.teamcode.teleop.prepPositions.OuttakeTest.outtakeConfig
import kotlin.math.abs

@TeleOp
class CanonEventTesting : LinearOpMode(){
    @Config
    data object CanonEventConfig {
        @JvmField var intakePos = 0.0
        @JvmField var shooterPos = 0.0
        @JvmField var multiplier = 1.0
        @JvmField var shooterOffset = 100.0
        @JvmField var intakeOffset = 0.0

        @JvmField var shooterTargetRpm = 0
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))

        var lastTime = now()
        var lastResetTime = now()
        val timeKeep = TimeKeep()
        var rpm = 0

        val intakeRight = ButtonReader { gamepad2.a }
        val intakeLeft = ButtonReader { gamepad2.b}
        val shootRight = ButtonReader { gamepad2.y}
        val shootLeft = ButtonReader { gamepad2.x}
        val fingerUp = ButtonReader {gamepad2.dpad_up}
        val fingerDown = ButtonReader {gamepad2.dpad_down}
        val highRpm = ButtonReader {gamepad2.left_bumper}
        val lowRpm = ButtonReader {gamepad2.right_bumper}
        val buttons = listOf(intakeRight, intakeLeft, shootRight, shootLeft, fingerUp, fingerDown, highRpm, lowRpm)

        robot.transfer.finger.position = 1.0

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            val currentTime = now()
            val dt = currentTime - lastTime
            buttons.forEach { it.readValue() }

            if (currentTime - lastResetTime >= outtakeConfig.sampleWindow) {
                val pos = robot.shooter.rpm
                val elapsed = currentTime - lastResetTime
                val revs = pos / outtakeConfig.TICKS_PER_REV
                rpm = abs((revs / elapsed) * 60.0).toInt()

                robot.shooter.updateRpm(rpm)

                lastResetTime = currentTime
            }

            /// Drive

            robot.drive.isSlowMode = gamepad1.right_trigger >= 0.2
            robot.drive.driveFieldCentric(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble()
            )
            if (gamepad1.y) {
                robot.drive.resetFieldCentric()
            }

            /// Transfer

            if (fingerUp.wasJustPressed()) /// finger
                robot.transfer.finger.position= 0.6

            if (fingerDown.wasJustPressed())
                robot.transfer.finger.position = 0.9

            if (intakeRight.wasJustPressed()){
                CanonEventConfig.multiplier ++
                robot.transfer.goToPos(CanonEventConfig.intakePos, CanonEventConfig.multiplier,CanonEventConfig.intakeOffset)
            }

            if (intakeLeft.wasJustPressed()){
                CanonEventConfig.multiplier --
                robot.transfer.goToPos(CanonEventConfig.shooterPos, CanonEventConfig.multiplier,CanonEventConfig.intakeOffset)
            }

            if (shootRight.wasJustPressed()){
                CanonEventConfig.multiplier ++
                robot.transfer.goToPos(CanonEventConfig.shooterPos, CanonEventConfig.multiplier,CanonEventConfig.shooterOffset)
            }

            if (shootLeft.wasJustPressed()){
                CanonEventConfig.multiplier --
                robot.transfer.goToPos(CanonEventConfig.shooterPos, CanonEventConfig.multiplier,CanonEventConfig.shooterOffset)
            }

            /// Shooter

            if (highRpm.wasJustPressed()){ /// shoot far
                robot.shooter.goToRmp(CanonEventConfig.shooterTargetRpm)
            }
            else if (lowRpm.wasJustPressed()) { /// shoot close
                robot.shooter.goToRmp(2000)
            }

            /// Intake

            if (gamepad1.left_bumper) {
                robot.intake.power = 1.0
            }
            else if (gamepad1.right_bumper) {
                robot.intake.power = -1.0
            }
            else {
                robot.intake.power = 0.0
            }

            telemetry.addData("a was pressed (set)", gamepad1.a)
            telemetry.addData("x was pressed (left)", gamepad1.x)
            telemetry.addData("b was pressed (right)", gamepad1.b)
            telemetry.addData("up was pressed (set)", gamepad1.dpad_up)
            telemetry.addData("down was pressed (left)", gamepad1.dpad_down)
            telemetry.addData("shooter power", robot.shooter.power)
            telemetry.addData("rpm", rpm)
            telemetry.addData("target rpm", robot.shooter.targetRpm)
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


        }
    }
}