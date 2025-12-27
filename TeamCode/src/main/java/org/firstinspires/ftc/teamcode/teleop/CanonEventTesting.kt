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
        @JvmField var ticksPerRev = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0)
        @JvmField var pos = ticksPerRev / 3.0
        @JvmField var multiplier = 0.0
        @JvmField var shooterOffset = 110.0
        @JvmField var intakeOffset = 0.0
        @JvmField var sampleWindow = 0.1
        @JvmField var TICKS_PER_REV = 8192.0

        @JvmField var shooterTargetRpm = 0
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))

        var lastTime = now()
        var lastResetTime = now()
        val timeKeep = TimeKeep()
        var rpm = 0.0
        var lastPos : Boolean = false // false = intake true = shooter

        val intakeRight = ButtonReader { gamepad2.y}
        val intakeLeft = ButtonReader { gamepad2.a}
        val shootRight = ButtonReader { gamepad2.b}
        val shootLeft = ButtonReader { gamepad2.x}
        val fingerUp = ButtonReader {gamepad2.dpad_up}
        val fingerDown = ButtonReader {gamepad2.dpad_down}
        val highRpm = ButtonReader {gamepad2.right_bumper}
        val lowRpm = ButtonReader {gamepad2.left_bumper}
        val buttons = listOf(intakeRight, intakeLeft, shootRight, shootLeft, fingerUp, fingerDown, highRpm, lowRpm)

        robot.transfer.finger.position = 1.0

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            val currentTime = now()
            buttons.forEach { it.readValue() }

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
                robot.transfer.finger.position= 0.5

            if (fingerDown.wasJustPressed())
                robot.transfer.finger.position = 0.95

            if (intakeRight.wasJustPressed()){
                    CanonEventConfig.multiplier--
                robot.transfer.goToPos(CanonEventConfig.pos, CanonEventConfig.multiplier,CanonEventConfig.intakeOffset)
                lastPos = false
            }

            if (intakeLeft.wasJustPressed()){
                    CanonEventConfig.multiplier++
                robot.transfer.goToPos(CanonEventConfig.pos, CanonEventConfig.multiplier,CanonEventConfig.intakeOffset)
                lastPos = false
            }

            if (shootRight.wasJustPressed()){
                if(lastPos) {
                    CanonEventConfig.multiplier--
                }
                robot.transfer.goToPos(CanonEventConfig.pos, CanonEventConfig.multiplier,CanonEventConfig.shooterOffset)
                lastPos = true
            }
            if (shootLeft.wasJustPressed()){
                if (lastPos) {
                    CanonEventConfig.multiplier++
                }
                robot.transfer.goToPos(CanonEventConfig.pos, CanonEventConfig.multiplier,CanonEventConfig.shooterOffset)
                lastPos = true
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

            /// Shooter
            //TODO: DON'T CALCULATE RPMS YOURSELF, USE THE VELOCITY PROVIDED BY THE MOTOR
            /*if (currentTime - lastResetTime >= CanonEventConfig.sampleWindow) {
                val pos = robot.shooter.encoder.getPositionAndVelocity().position
                val elapsed = currentTime - lastResetTime
                val revs = pos / CanonEventConfig.TICKS_PER_REV
                robot.shooter.rpm = ((revs / elapsed) * 60.0)

                //robot.shooter.motorBottom.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                //robot.shooter.motorBottom.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

                lastResetTime = currentTime
            }*/

            if (highRpm.wasJustPressed()){ /// shoot far
                robot.shooter.goToRmp(CanonEventConfig.shooterTargetRpm.toDouble())
            }
            else if (lowRpm.wasJustPressed()) { /// shoot close
                robot.shooter.goToRmp(2500.0)
            }

            robot.shooter.update(timeKeep.deltaTime)

            telemetry.addData("a was pressed (set)", gamepad1.a)
            telemetry.addData("x was pressed (left)", gamepad1.x)
            telemetry.addData("b was pressed (right)", gamepad1.b)
            telemetry.addData("up was pressed (set)", gamepad1.dpad_up)
            telemetry.addData("down was pressed (left)", gamepad1.dpad_down)
            telemetry.addData("shooter power", robot.shooter.power)
            telemetry.addData("rpm", robot.shooter.rpm)
            telemetry.addData("target rpm", robot.shooter.targetRpm)
            telemetry.addData("pos", robot.transfer.position)
            telemetry.addData("fingir pos", robot.transfer.finger.position)
            telemetry.addData("target pos", robot.transfer.targetPosition)
            telemetry.addData("power trans", robot.transfer.power)
            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.addData("multiplier", CanonEventConfig.multiplier)
            telemetry.update()


            robot.transfer.update(timeKeep.deltaTime)
            lastTime = currentTime

        }
    }
}