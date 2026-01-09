package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.VelConstraint
import com.commonlibs.units.Pose
import com.commonlibs.units.SleepAction
import com.commonlibs.units.deg
import com.commonlibs.units.inch
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.robot.Robot

@Autonomous
@Disabled
class SmallTriangleBlue : LinearOpMode() {
    val startPose = Pose(63.inch, -11.inch, 180.0.deg)
    val smallTrianglePose = Pose(55.inch, -10.inch, 202.0.deg)
    val bigTrianglePose = Pose(-10.inch, -10.inch, 220.0.deg)

    val rightIntakePose = Pose(36.inch, -30.inch, 270.0.deg)
    val middleIntakePose = Pose(12.inch, -30.inch, 270.0.deg)
    val leftIntakePose = Pose(-12.inch, -30.inch, 270.0.deg)

    val endPose = Pose(58.inch, -28.inch, 180.0.deg)

    val rpmFar = 3260.0
    val rpmClose = 2490.0

    val shooterOffset = 94.0

    var ticksPerRev = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0)
    var pos = ticksPerRev / 3.0

    override fun runOpMode() {
        val robot = Robot(hardwareMap, startPose)
        val timeKeep = TimeKeep()

        robot.transfer.fingerDown()

        val kinematics = MecanumKinematics(
            15.0,
            1.0
        )

        val slowSpeed: VelConstraint = MinVelConstraint(
            listOf(
                kinematics.WheelVelConstraint(5.0),
                AngularVelConstraint(Math.toRadians(180.0))
            )
        )

        val actionPGP = SequentialAction(
            robot.drive.actionBuilder(startPose)
                .strafeToLinearHeading(smallTrianglePose)
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmFar))
                .build(),
            robot.drive.correctionAction(smallTrianglePose, 0.75.s),

            robot.shootBalls(rpmFar, 0),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(smallTrianglePose)
                .strafeToLinearHeading(rightIntakePose)
                .setTangent(90.deg)
                .afterTime(0.s, robot.intakeBalls(0, 0))
                .lineToY(-42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmFar))
                .strafeToLinearHeading(smallTrianglePose)
                .build(),

            robot.shootBalls(rpmFar, 0),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(smallTrianglePose)
                .strafeToLinearHeading(middleIntakePose)
                .setTangent(90.deg)
                .afterTime(0.s, robot.intakeBalls(0,0))
                .lineToY(-42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmClose))
                .strafeToLinearHeading(bigTrianglePose)
                .build(),

            robot.shootBalls(rpmClose, 1),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(leftIntakePose)
                .setTangent(90.deg)
                .afterTime(0.s, robot.intakeBalls(0,0))
                .lineToY(-42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .strafeToLinearHeading(bigTrianglePose)
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmClose))
                .build(),

            robot.shootBalls(rpmClose, 2),

            robot.drive.actionBuilder(smallTrianglePose)
                .strafeTo(endPose.position)
                .build()
        )

        val actionPPG = SequentialAction(
            robot.drive.actionBuilder(startPose)
                .strafeToLinearHeading(smallTrianglePose)
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmFar))
                .build(),
            robot.drive.correctionAction(smallTrianglePose, 0.75.s),
            robot.shootBalls(rpmFar, 2),


            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(smallTrianglePose)
                .strafeToLinearHeading(rightIntakePose)
                .setTangent(90.deg)
                .afterTime(0.s, robot.intakeBalls(0,0))
                .lineToY(-42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmFar))
                .strafeToLinearHeading(smallTrianglePose)
                .build(),
            robot.shootBalls(rpmFar,2),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(smallTrianglePose)
                .strafeToLinearHeading(middleIntakePose)
                .setTangent(90.deg)
                .afterTime(0.s, robot.intakeBalls(0,0))
                .lineToY(-42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmClose))
                .strafeToLinearHeading(bigTrianglePose)
                .build(),
            robot.shootBalls(rpmClose, 0),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(leftIntakePose)
                .setTangent(90.deg)
                .afterTime(0.s, robot.intakeBalls(0,0))
                .lineToY(-42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .strafeToLinearHeading(bigTrianglePose)
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmClose))
                .build(),
            robot.shootBalls(rpmClose,1),

            robot.drive.actionBuilder(smallTrianglePose)
                .strafeTo(endPose.position)
                .build()
        )

        val actionGPP = SequentialAction(
            robot.drive.actionBuilder(startPose)
                .strafeToLinearHeading(smallTrianglePose)
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmFar))
                .build(),
            robot.drive.correctionAction(smallTrianglePose, 0.75.s),

            robot.shootBalls(rpmFar, 0),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(smallTrianglePose)
                .strafeToLinearHeading(rightIntakePose)
                .setTangent(90.deg)
                .afterTime(0.s, robot.intakeBalls(0,0))
                .lineToY(-42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmFar))
                .strafeToLinearHeading(smallTrianglePose)
                .build(),
            robot.drive.correctionAction(smallTrianglePose, 0.25.s),

            robot.shootBalls(rpmClose, 2),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(smallTrianglePose)
                .strafeToLinearHeading(middleIntakePose)
                .setTangent(90.deg)
                .afterTime(0.s, robot.intakeBalls(0,0))
                .lineToY(-42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmClose))
                .strafeToLinearHeading(bigTrianglePose)
                .build(),

            robot.shootBalls(rpmClose, 1),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(leftIntakePose)
                .setTangent(90.deg)
                .afterTime(0.s, robot.intakeBalls(0,0))
                .lineToY(-42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .strafeToLinearHeading(bigTrianglePose)
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmClose))
                .build(),

            robot.shootBalls(rpmClose, 2),

            robot.drive.actionBuilder(smallTrianglePose)
                .strafeTo(endPose.position)
                .build()
        )

        val startAction =  InstantAction {
            robot.drive.actionBuilder(startPose)
                .strafeToLinearHeading(smallTrianglePose)}

        while (opModeInInit()) {
            telemetry.addData("case id", robot.camera.detectAprilTagCase())
            telemetry.update()
            sleep(20)
        }

        val action = SequentialAction(
            startAction,
            when (robot.camera.detectAprilTagCase()) {
                21 -> actionGPP
                22 -> actionPGP
                else -> actionPPG
            }
        )

        val dash = FtcDashboard.getInstance()
        val c = Canvas()
        action.preview(c)

        var running = true

        telemetry.addData("True case: ",robot.camera.detectAprilTagCase())

        while (running && opModeIsActive()) {
            timeKeep.resetDeltaTime()
            robot.transfer.update(timeKeep.deltaTime)
            robot.shooter.update(timeKeep.deltaTime)

            val packet = TelemetryPacket()
            packet.fieldOverlay().operations.addAll(c.operations)


            running = action.run(packet)

            dash.sendTelemetryPacket(packet)

            telemetry.addData("case id", robot.camera.detectAprilTagCase())
            telemetry.addData("color", robot.camera.colorSensor.getAnalysis())
            telemetry.addData("rpm", robot.shooter.rpm)
            telemetry.update()
        }
    }
}