package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Trajectory
import com.commonlibs.units.Pose
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.commonlibs.units.inch
import com.commonlibs.units.pose
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.library.buttons.ButtonReader
import org.firstinspires.ftc.teamcode.robot.Robot
import java.util.concurrent.TimeUnit

@TeleOp
class InterestingDrive : LinearOpMode(){
    @Config
    data object InterestingDriveConfig {
        @JvmField var ticksPerRev = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0)
        @JvmField var pos = ticksPerRev / 3.0
        @JvmField var multiplier = 0
        @JvmField var shooterOffset = 94.0
        @JvmField var intakeOffset = 0.0
        @JvmField val rpmFar = 3260.0
        @JvmField val rpmClose = 2490.0
    }

    val smallTrianglePose = Pose(55.inch, -10.inch, 203.0.deg)

    var intakeAction : Action? = null
    var shootAction : Action? = null

    var driveAction : Action? = null

    var currTrajectory: Trajectory? = null
    var goingToTarget = false

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))
        val timeKeep = TimeKeep()
        var shootOffset : Boolean = false // false = intake true = shooter

        val shootRight = ButtonReader { gamepad2.b}
        val shootLeft = ButtonReader { gamepad2.x}
        val highRpm = ButtonReader {gamepad2.right_bumper}
        val lowRpm = ButtonReader {gamepad2.left_bumper}
        val stopShooter = ButtonReader {gamepad2.dpad_left}
        val dpadRight = ButtonReader {gamepad2.dpad_right}
        val stopButton1 = ButtonReader { gamepad1.touchpad }
        val spew = ButtonReader {gamepad1.right_trigger >= 0.2}
        val stopButton2 = ButtonReader {gamepad2.touchpad}
        val snipe = ButtonReader {gamepad1.x}


        val buttons = listOf(shootRight, shootLeft, highRpm, lowRpm, stopShooter, dpadRight, spew, stopButton2, snipe)


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
            robot.drive.updatePoseEstimate()


            if (gamepad1.y) {
                robot.drive.resetFieldCentric()
            }
            robot.drive.isSlowMode = gamepad1.right_trigger >= 0.2
            robot.drive.driveFieldCentric(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble()
            )
            if (snipe.wasJustPressed()) {

                val allowedIds = setOf(20, 24) // change this to the exact IDs you want (e.g. setOf(20,24))

                driveAction = object : Action {

                    // === TUNABLES ===
                    private val kP = 0.0025          // proportional gain
                    private val maxRot = 0.7         // max rotation power
                    private val deadbandPx = 50    // pixels from center considered "aligned"
                    private val imageCenterX = 1280.0 / 2.0  // use your aprilTag camera resolution / 2

                    override fun run(p: TelemetryPacket): Boolean {
                        // get all detections and filter to allowed IDs
                        val all = robot.camera.aprilTag.detections
                        val filtered = all.filter { det ->
                            try {
                                allowedIds.contains(det.id)
                            } catch (e: Exception) {
                                false
                            }
                        }

                        p.put("allTags", all.size)
                        p.put("allowedTags", filtered.size)

                        // if no allowed tag -> keep scanning (rotate slowly)
                        if (filtered.isEmpty()) {
                            robot.drive.driveFieldCentric(0.0, 0.0, 0.25)
                            p.put("state", "scanning")
                            return true
                        }

                        // use the first allowed detection (you can pick the largest/closest if you prefer)
                        val tag = filtered[0]
                        val tagX = try { tag.center.x } catch (e: Exception) { imageCenterX }
                        val errorPx = tagX - imageCenterX

                        p.put("tagId", tag.id)
                        p.put("tagX", tagX)
                        p.put("errorPx", errorPx)

                        // If horizontally centered -> stop
                        if (kotlin.math.abs(errorPx) < deadbandPx) {
                            robot.drive.driveFieldCentric(0.0, 0.0, 0.0)
                            p.put("state", "LOCKED")
                            return false // action complete
                        }

                        // Proportional rotation
                        var rot = errorPx * kP
                        rot = rot.coerceIn(-maxRot, maxRot)

                        robot.drive.driveFieldCentric(0.0, 0.0, rot)
                        p.put("rotCmd", rot)

                        return true
                    }
                    override fun preview(c: com.acmerobotics.dashboard.canvas.Canvas) { /* no preview */ }
                }
            }

            /// Transfer

            if (shootRight.wasJustPressed()){
                if(shootOffset) {
                    InterestingDriveConfig.multiplier--
                    if (InterestingDriveConfig.multiplier == -1) {
                        InterestingDriveConfig.multiplier = 2
                    }
                }
                robot.transfer.goToPos(InterestingDriveConfig.pos, InterestingDriveConfig.multiplier,InterestingDriveConfig.shooterOffset)
                shootOffset = true
            }
            if (shootLeft.wasJustPressed()){
                if (shootOffset) {
                    InterestingDriveConfig.multiplier++
                    if (InterestingDriveConfig.multiplier == 3) {
                        InterestingDriveConfig.multiplier = 0
                    }
                }
                robot.transfer.goToPos(InterestingDriveConfig.pos, InterestingDriveConfig.multiplier,InterestingDriveConfig.shooterOffset)
                shootOffset = true
            }

            /// Shooter

            if (highRpm.wasJustPressed()) {
                shootAction = robot.shootTeleBalls(InterestingDriveConfig.rpmFar, InterestingDriveConfig.multiplier)
            }
            else if (lowRpm.wasJustPressed()) {
                shootAction = robot.shootTeleBalls(InterestingDriveConfig.rpmClose, InterestingDriveConfig.multiplier)
            }
            else if (stopShooter.wasJustPressed()) { /// stop shoot
                robot.shooter.goToRmp(0.0)
            }

            if (gamepad1.dpad_left) {
                robot.transfer.power = -0.1
            } else if (gamepad1.dpad_right) {
                robot.transfer.power = 0.1
            } else {
                robot.transfer.power = 0.0
            }

            if (gamepad1.dpad_up) {
                robot.transfer.resetPos()
            }


            /// Intake

            if(dpadRight.wasJustPressed()) {
                intakeAction = robot.intakeTeleBalls(0)
                shootOffset = false
            }
            else if(spew.wasJustPressed()) {
                intakeAction = robot.intake.spew()
            }

            /// Stop

            if (stopButton2.wasJustPressed()) {
                shootAction = null
                intakeAction = null
                driveAction = null
            }
            if (stopButton1.wasJustPressed()) {
                driveAction = null
            }
            runActions()
            telemetry.addData("Best Match", robot.camera.sensorColor.closestSwatch)
            telemetry.addData("pos", robot.transfer.position)
            telemetry.addData("target pos", robot.transfer.targetPosition)
            telemetry.addData("power trans", robot.transfer.power)
            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.addData("multiplier", InterestingDriveConfig.multiplier)
            telemetry.addData("action", intakeAction)
            telemetry.update()

            robot.shooter.update(timeKeep.deltaTime)
            robot.transfer.update(timeKeep.deltaTime)
        }
    }

    private fun runActions() {
        intakeAction?.let {
            val p = TelemetryPacket()
            if (!it.run(p)) {
                intakeAction = null
            }
            FtcDashboard.getInstance().sendTelemetryPacket(p)
        }

        shootAction?.let {
            val p = TelemetryPacket()
            if (!it.run(p)) {
                shootAction = null
            }
            FtcDashboard.getInstance().sendTelemetryPacket(p)
        }

        driveAction?.let {
            val p = TelemetryPacket()
            if (!it.run(p)) {
                // action finished
                driveAction = null
                goingToTarget = false
            }
            FtcDashboard.getInstance().sendTelemetryPacket(p)
        }
    }
}