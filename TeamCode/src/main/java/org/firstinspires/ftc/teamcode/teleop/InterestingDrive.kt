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
        @JvmField val rpmClose = 2700.0
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
        val spew = ButtonReader {gamepad1.left_trigger >= 0.2}
        val stopButton = ButtonReader {gamepad2.touchpad}
        val snipe = ButtonReader {gamepad1.x}


        val buttons = listOf(shootRight, shootLeft, highRpm, lowRpm, stopShooter, dpadRight, spew, stopButton, snipe)


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
            // call early in loop
            robot.drive.updatePoseEstimate()

// ----------------- CONFIG -----------------
            val JOY_CANCEL_DEADBAND = 0.08       // if any stick magnitude > this -> cancel snipe
            val SCAN_ROT_SPEED = 0.15           // slow scan rotation when no tag seen
            val kP = 0.002                       // proportional gain (pixels -> rot)
            val maxRot = 0.3








            // max rotation power while homing
            val deadbandPx = 1                  // pixels from center considered aligned
            val imageCenterX = 1280.0 / 2.0     // adjust to your camera width/2
            val allowedIds = setOf(20, 24)      // change to the exact IDs you want
// -------------------------------------------

            fun driverMoved(): Boolean {
                return kotlin.math.abs(gamepad1.left_stick_x)  > JOY_CANCEL_DEADBAND ||
                        kotlin.math.abs(gamepad1.left_stick_y)  > JOY_CANCEL_DEADBAND ||
                        kotlin.math.abs(gamepad1.right_stick_x) > JOY_CANCEL_DEADBAND ||
                        kotlin.math.abs(gamepad1.right_trigger) >= 0.2 ||
                        kotlin.math.abs(gamepad1.left_trigger)  >= 0.2
            }

            if (!goingToTarget) {
                // regular manual control
                if (gamepad1.y) {
                    robot.drive.resetFieldCentric()
                }
                robot.drive.isSlowMode = gamepad1.right_trigger >= 0.2
                robot.drive.driveFieldCentric(
                    -gamepad1.left_stick_y.toDouble(),
                    -gamepad1.left_stick_x.toDouble(),
                    -gamepad1.right_stick_x.toDouble()
                )

                // start snipe action
                if (snipe.wasJustPressed()) {
                    driveAction = object : Action {
                        override fun run(p: TelemetryPacket): Boolean {
                            val all = robot.camera.aprilTag.detections ?: emptyList()
                            val filtered = all.filter { det -> det?.id != null && allowedIds.contains(det.id) }

                            p.put("allTags", all.size)
                            p.put("allowedTags", filtered.size)

                            if (filtered.isEmpty()) {
                                robot.drive.driveFieldCentric(0.0, 0.0, SCAN_ROT_SPEED)
                                p.put("state", "scanning")
                                return false
                            }

                            val tag = filtered[0]
                            val tagX = try { tag.center.x } catch (e: Exception) { imageCenterX }
                            val errorPx = -(tagX - imageCenterX)

                            p.put("tagId", tag.id)
                            p.put("tagX", tagX)
                            p.put("errorPx", errorPx)

                            // If horizontally centered -> stop
                            if (kotlin.math.abs(errorPx) < deadbandPx) {
                                robot.drive.driveFieldCentric(0.0, 0.0, 0.0)
                                p.put("state", "LOCKED")
                                return false // action complete
                            }

                            // Proportional rotation toward center
                            var rot = errorPx * kP
                            rot = rot.coerceIn(-maxRot, maxRot)

                            robot.drive.driveFieldCentric(0.0, 0.0, rot)
                            p.put("rotCmd", rot)
                            p.put("state", "homing")
                            return true
                        }

                        override fun preview(c: com.acmerobotics.dashboard.canvas.Canvas) { /* no preview */ }
                    }

                    // mark that we are now running the action (it will be executed next block)
                    goingToTarget = true
                }
            }

// If an action is active, run it and allow joystick override
            if (goingToTarget) {
                // immediate driver cancel
                if (driverMoved()) {
                    driveAction = null
                    goingToTarget = false
                } else {
                    driveAction?.let {
                        val p = TelemetryPacket()
                        if (!it.run(p)) {
                            // action finished normally
                            driveAction = null
                            goingToTarget = false
                        }
                        FtcDashboard.getInstance().sendTelemetryPacket(p)
                    }
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

            if (stopButton.wasJustPressed()) {
                shootAction = null
                intakeAction = null
            }
            runActions()
            telemetry.addData("Best Match", robot.camera.sensorColor.closestSwatch)
            telemetry.addData("pos", robot.transfer.position)
            telemetry.addData("target pos", robot.transfer.targetPosition)
            telemetry.addData("power trans", robot.transfer.power)
            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.addData("multiplier", InterestingDriveConfig.multiplier)
            telemetry.addData("action intake: ", intakeAction)
            telemetry.addData("action drive: ", driveAction)
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
    }
}