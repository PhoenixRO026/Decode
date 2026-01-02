package org.firstinspires.ftc.teamcode.teleop

/*
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
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.library.buttons.ButtonReader
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor
import java.util.concurrent.TimeUnit



@TeleOp
class JustDrive : LinearOpMode(){
    @Config
    data object JustDriveConfig {
        @JvmField var ticksPerRev = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0)
        @JvmField var pos = ticksPerRev / 3.0
        @JvmField var multiplier = 0.0
        @JvmField var shooterOffset = 94.0
        @JvmField var intakeOffset = 0.0
        @JvmField var shooterTargetRpm = 3400
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))
        val timeKeep = TimeKeep()
        var lastPos : Boolean = false // false = intake true = shooter


        val intakeRight = ButtonReader { gamepad2.y}
        val intakeLeft = ButtonReader { gamepad2.a}
        val shootRight = ButtonReader { gamepad2.b}
        val shootLeft = ButtonReader { gamepad2.x}
        val fingerUp = ButtonReader {gamepad2.dpad_up}
        val fingerDown = ButtonReader {gamepad2.dpad_down}
        val highRpm = ButtonReader {gamepad2.right_bumper}
        val lowRpm = ButtonReader {gamepad2.left_bumper}
        val stopShooter = ButtonReader {gamepad2.dpad_left}
        val buttons = listOf(intakeRight, intakeLeft, shootRight, shootLeft, fingerUp, fingerDown, highRpm, lowRpm, stopShooter)

        robot.transfer.finger.position = 1.0

        while (opModeInInit()){
            robot.camera.portal.getProcessorEnabled(robot.camera.colorSensor)
        }

        val exposureCtrl = robot.camera.portal.getCameraControl(ExposureControl::class.java)
        exposureCtrl.setExposure(CameraConfig.desiredExposureMs, TimeUnit.MILLISECONDS)

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
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

            */
/* if (fingerUp.wasJustPressed())
                 robot.transfer.finger.position = 0.5

             if (fingerDown.wasJustPressed())
                 robot.transfer.finger.position = 0.95*//*



            if (fingerUp.wasJustPressed()){
                robot.transfer.fingerUp()
                sleep(1000)
                robot.transfer.fingerDown()
            }


            //if (fingerDown.wasJustPressed())


            if (intakeRight.wasJustPressed()){
                JustDriveConfig.multiplier--
                robot.transfer.goToPos(JustDriveConfig.pos, JustDriveConfig.multiplier, JustDriveConfig.intakeOffset)
                lastPos = false
            }

            if (intakeLeft.wasJustPressed()){
                JustDriveConfig.multiplier++
                robot.transfer.goToPos(JustDriveConfig.pos, JustDriveConfig.multiplier, JustDriveConfig.intakeOffset)
                lastPos = false
            }

            if (shootRight.wasJustPressed()){
                if(lastPos) {
                    JustDriveConfig.multiplier--
                }
                robot.transfer.goToPos(JustDriveConfig.pos, JustDriveConfig.multiplier,JustDriveConfig.shooterOffset)
                lastPos = true
            }
            if (shootLeft.wasJustPressed()){
                if (lastPos) {
                    JustDriveConfig.multiplier++
                }
                robot.transfer.goToPos(JustDriveConfig.pos, JustDriveConfig.multiplier,JustDriveConfig.shooterOffset)
                lastPos = true
            }

            /// Intake

            if (gamepad2.right_trigger > 0.1) {
                robot.intake.power = 1.0
            }
            else if (gamepad2.left_trigger > 0.1) {
                robot.intake.power = -1.0
            }
            else {
                robot.intake.power = 0.0
            }


            if (highRpm.wasJustPressed()){ /// shoot far
                robot.shooter.goToRmp(JustDriveConfig.shooterTargetRpm.toDouble())
            }
            else if (lowRpm.wasJustPressed()) { /// shoot close
                robot.shooter.goToRmp(2500.0)
            }
            else if (stopShooter.wasJustPressed()) { /// stop shoot
                robot.shooter.goToRmp(0.0)
            }

            robot.shooter.update(timeKeep.deltaTime)

            val result: PredominantColorProcessor.Result = robot.camera.colorSensor.getAnalysis()


            telemetry.addData("Best Match", result.closestSwatch)

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
            telemetry.addData("multiplier", JustDriveConfig.multiplier)
            telemetry.update()

            robot.transfer.update(timeKeep.deltaTime)
        }
    }
}*/
