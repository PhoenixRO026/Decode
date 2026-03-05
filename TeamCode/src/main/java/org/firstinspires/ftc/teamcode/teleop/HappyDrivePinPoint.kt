//package org.firstinspires.ftc.teamcode.teleop
//
//import com.acmerobotics.dashboard.FtcDashboard
//import com.acmerobotics.dashboard.config.Config
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket
//import com.acmerobotics.roadrunner.Action
//import com.acmerobotics.roadrunner.InstantAction
//import com.acmerobotics.roadrunner.SequentialAction
//import com.commonlibs.units.Pose
//import com.commonlibs.units.angle
//import com.commonlibs.units.cm
//import com.commonlibs.units.deg
//import com.commonlibs.units.radsec
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import org.firstinspires.ftc.teamcode.library.TimeKeep
//import org.firstinspires.ftc.teamcode.library.buttons.ButtonReader
//import org.firstinspires.ftc.teamcode.library.buttons.ToggleButtonReader
//import org.firstinspires.ftc.teamcode.robot.Robot
//import org.firstinspires.ftc.teamcode.robot.Shooter.MODE
//import org.firstinspires.ftc.teamcode.robot.Spindexer
//
//
//@TeleOp
//open class HappyDrivePinPoint : LinearOpMode(){
//    open val pipeline: Int = 1
//    @Config
//    data object HappyDrive {
//        @JvmField var rpmSmall = 3300.0
//        @JvmField var rpmBig = 2975.0
//
//        @JvmField var rpmRest = 1200.0
//
//    }
//    private var driver1Action: Action? = null
//    private var driver1ActionIsIntake: Boolean = false
//
//    override fun runOpMode() {
//        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
//
//        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))
//        val timeKeep = TimeKeep()
//
//        val targetX = 72.0  // field X coordinate of your target
//        val targetY = -36.0 // field Y coordinate of your target
//
//        robot.limelight.setPipeline(pipeline)
//
//        val shootGreen = ButtonReader { gamepad2.x }
//        val shootPurple = ButtonReader { gamepad2.b }
//        val shootAll = ButtonReader { gamepad2.y }
//        val highRpm = ButtonReader {gamepad2.right_bumper}
//        val lowRpm = ButtonReader {gamepad2.left_bumper}
//        val stopShooter = ButtonReader {gamepad2.dpad_left}
//        val restShooter = ButtonReader {gamepad2.dpad_right}
//        val intakeBalls = ToggleButtonReader ({gamepad1.x})
//        val buttons = listOf(shootGreen, shootPurple, shootAll, highRpm, lowRpm, stopShooter, intakeBalls)
//
//        waitForStart()
//
//        robot.transfer.fingerDown()
//        robot.transfer.servoTransfer1.position = 0.0400
//        robot.transfer.servoTransfer2.position = 0.0400
//
//        while (opModeIsActive()) {
//            timeKeep.resetDeltaTime()
//            buttons.forEach { it.readValue() }
//            val robotVel = robot.drive.updatePoseEstimateOdo()
//
//            /// Drive
//
//            if (gamepad1.left_trigger >= 0.2) {
//                robot.drive.isSlowMode = true
//            } else {
//                robot.drive.isSlowMode = false
//            }
//
//            robot.drive.driveFieldCentric(
//                -gamepad1.left_stick_y.toDouble(),
//                -gamepad1.left_stick_x.toDouble(),
//                -gamepad1.right_stick_x.toDouble()
//            )
//            if (gamepad1.y) {
//                robot.drive.resetFieldCentric()
//            }
//
//            /// Intake
//            if (intakeBalls.state) {
//                // If toggle turned on, and no current action, start a NEW intake action
//                if (driver1Action == null) {
//                    driver1Action = SequentialAction(
//                        robot.intakeTeleBalls(),
//                        InstantAction { intakeBalls.setState(false) }
//                    )
//                    driver1ActionIsIntake = true
//                }
//            } else {
//                if (driver1ActionIsIntake) {
//                    driver1Action = null
//                    driver1ActionIsIntake = false
//                    // stop intake motors immediately (safe fallback)
//                    robot.intake.power = 0.0
//                }
//
//                /// Intake
//                if (gamepad1.right_bumper) {
//                    robot.intake.power = 1.0
//                } else if (gamepad1.left_bumper) {
//                    robot.intake.power = -1.0
//                } else {
//                    robot.intake.power = 0.0
//                }
//
//                if (shootGreen.wasJustPressed() && driver1Action == null) {
//                    driver1Action = robot.shootGreen()
//                }
//                if (shootPurple.wasJustPressed() && driver1Action == null) {
//                    driver1Action = robot.shootPurple()
//                }
//                if (shootAll.wasJustPressed() && driver1Action == null) {
//                    driver1Action = robot.shootBallsTele()
//                }
//            }
//
//            if (highRpm.wasJustPressed()) { /// shoot far
//                robot.shooter.goToRmp(HappyDrive.rpmSmall)
//            } else if (lowRpm.wasJustPressed()) { /// shoot close
//                robot.shooter.goToRmp(HappyDrive.rpmBig)
//            } else if (restShooter.wasJustPressed()) { /// stop shoot
//                robot.shooter.goToRmp(HappyDrive.rpmRest)
//            } else if (stopShooter.wasJustPressed()) { /// stop shoot
//                robot.shooter.goToRmp(0.0)
//            }
//
//            if (gamepad2.left_trigger > 0) {
//                robot.shooter.currentMode = MODE.MANUAL
//                robot.shooter.powerTurret = -0.5
//            }
//            else if (gamepad2.right_trigger > 0) {
//                robot.shooter.currentMode = MODE.MANUAL
//                robot.shooter.powerTurret = 0.5
//            }
//            else if (robot.shooter.currentMode != MODE.PID) {
//                robot.shooter.currentMode = MODE.PID
//                robot.shooter.powerTurret = 0.0
//                robot.shooter.resetTargetAngle(robot.drive.mecanumDrive.localizer.pose.heading.angle)
//            }
//
//
//            robot.shooter.updateRpm(timeKeep.deltaTime)
//
//            // --- Update camera ---
//            robot.limelight.updateHeadingError()
//
//// --- Get robot pose ---
//            val pose = robot.drive.mecanumDrive.localizer.pose
//            val robotHeadingRad = pose.heading.toDouble()
//
//
//// --- Calculate vector to target ---
//            val dx = targetX - pose.position.x
//            val dy = targetY - pose.position.y
//            val angleToTargetWorld = Math.atan2(dy, dx)  // angle in radians
//
//            val pinpointError = Math.toDegrees(angleToTargetWorld - robotHeadingRad)
//
//            val finalHeadingError = if (robot.limelight.tagVisible) {
//                robot.limelight.headingErrorDeg  // trust camera directly when visible
//            } else {
//                pinpointError  // fall back to odometry-based angle
//            }
//
//
//            robot.shooter.updateTurret(
//                timeKeep.deltaTime,
//                finalHeadingError,
//                robotVel.angVel.radsec,
//                0.0
//            )
//
//            robot.shooter.addTelemetry(telemetry)
//
//            telemetry.addData("results", robot.limelight.camera.latestResult.fiducialResults)
//            telemetry.addData("robotangle", robot.drive.mecanumDrive.localizer.pose.heading.angle)
//            telemetry.addData("transfer pos", when (robot.transfer.currentPos) {
//                Spindexer.TransferPos.intake0 -> "intake0"
//                Spindexer.TransferPos.intake1 -> "intake1"
//                Spindexer.TransferPos.intake2 -> "intake2"
//                Spindexer.TransferPos.shoot1 -> "shoot1"
//                Spindexer.TransferPos.shoot2 -> "shoot2"
//                Spindexer.TransferPos.shoot0 -> "shoot0"
//                Spindexer.TransferPos.pseudo0 -> "pseudo0"
//                Spindexer.TransferPos.pseudo2 -> "pseudo2"
//            })
//            telemetry.addData("slot 0", robot.transfer.slots[0])
//            telemetry.addData("slot 1", robot.transfer.slots[1])
//            telemetry.addData("slot 2", robot.transfer.slots[2])
//
//            telemetry.addData("distance", robot.limelight.getDistance())
//            telemetry.addData("auto rpm", robot.shooter.rpm)
//
//            telemetry.addData("error heading", robot.limelight.headingErrorDeg)
//            telemetry.addData("turret power", robot.shooter.powerTurret)
//            telemetry.addData("target pos", robot.shooter.targetPos)
//
//
//            runActions()
//
//            telemetry.update()
//        }
//    }
//    private fun normalize(angle: Double): Double {
//        return Math.atan2(Math.sin(angle), Math.cos(angle))
//    }
//
//    private fun runActions() {
//        driver1Action?.let {
//            if (!it.run(TelemetryPacket())) {
//                driver1Action = null
//            }
//        }
//    }
//}