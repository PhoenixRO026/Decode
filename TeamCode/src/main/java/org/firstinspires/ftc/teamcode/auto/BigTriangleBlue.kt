package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AngularVelConstraint
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.robot.Robot

@Autonomous
class BigTriangleBlue : LinearOpMode() {
    val startPose = Pose(-60.inch, -38.inch, 90.0.deg)
    val smallTrianglePose = Pose(57.inch, -10.inch, 200.0.deg)
    val bigTrianglePose = Pose(-10.inch, -10.inch, 225.0.deg)

    val rightIntakePose = Pose(36.inch, -30.inch, 270.0.deg)
    val middleIntakePose = Pose(12.inch, -30.inch, 270.0.deg)
    val leftIntakePose = Pose(-12.inch, -30.inch, 270.0.deg)

    val endPose = Pose(10.inch, -58.inch, 220.0.deg)

    val rpmFar = 3300.0
    val rpmClose = 2500.0

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


        val actionGPP = SequentialAction(
            robot.drive.actionBuilder(startPose)
                .strafeToLinearHeading(bigTrianglePose)
                .build(),
            robot.drive.correctionAction(bigTrianglePose, 0.75.s),
            robot.shootBalls(rpmClose, 0),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(leftIntakePose)
                .setTangent(90.deg)
                .afterTime(0.s, robot.intakeBalls(0))
                .lineToY(-45.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmFar))
                .strafeToLinearHeading(bigTrianglePose)
                .build(),
            robot.shootBalls(rpmClose, 2),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(middleIntakePose)
                .setTangent(-90.deg)
                .afterTime(0.s, robot.intakeBalls(0))
                .lineToY(-45.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmClose))
                .strafeToLinearHeading(bigTrianglePose)
                .build(),
            robot.shootBalls(rpmClose, 1),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(rightIntakePose)
                .setTangent(-90.deg)
                .afterTime(0.s, robot.intakeBalls(0))
                .lineToY(-45.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .strafeToLinearHeading(smallTrianglePose)
                .build(),
            robot.shootBalls(rpmFar, 2),

            robot.drive.actionBuilder(smallTrianglePose)
                .strafeToLinearHeading(endPose)
                .build()
        )

        val actionPPG = SequentialAction(
            robot.drive.actionBuilder(startPose)
                .strafeToLinearHeading(bigTrianglePose)
                .build(),
            robot.drive.correctionAction(bigTrianglePose, 0.75.s),
            robot.shootBalls(rpmClose, 2),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(rightIntakePose)
                .setTangent(90.deg)
                .afterTime(0.s, robot.intakeBalls(0))
                .lineToY(-45.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmFar))
                .strafeToLinearHeading(bigTrianglePose)
                .build(),
            robot.shootBalls(rpmClose, 1),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(middleIntakePose)
                .setTangent(-90.deg)
                .afterTime(0.s, robot.intakeBalls(0))
                .lineToY(-45.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmClose))
                .strafeToLinearHeading(bigTrianglePose)
                .build(),
            robot.shootBalls(rpmClose, 0),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(rightIntakePose)
                .setTangent(-90.deg)
                .afterTime(0.s, robot.intakeBalls(0))
                .lineToY(-45.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .strafeToLinearHeading(smallTrianglePose)
                .build(),
            robot.shootBalls(rpmFar, 2),

            robot.drive.actionBuilder(smallTrianglePose)
                .strafeToLinearHeading(endPose)
                .build()
        )

        val actionPGP = SequentialAction(
            robot.drive.actionBuilder(startPose)
                .strafeToLinearHeading(bigTrianglePose)
                .build(),
            robot.drive.correctionAction(bigTrianglePose, 0.75.s),
            robot.shootBalls(rpmClose, 0),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(rightIntakePose)
                .setTangent(90.deg)
                .afterTime(0.s, robot.intakeBalls(0))
                .lineToY(-45.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmFar))
                .strafeToLinearHeading(bigTrianglePose)
                .build(),
            robot.shootBalls(rpmClose, 2),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(middleIntakePose)
                .setTangent(-90.deg)
                .afterTime(0.s, robot.intakeBalls(0))
                .lineToY(-45.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmClose))
                .strafeToLinearHeading(bigTrianglePose)
                .build(),
            robot.shootBalls(rpmClose, 1),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(rightIntakePose)
                .setTangent(-90.deg)
                .afterTime(0.s, robot.intakeBalls(0))
                .lineToY(-45.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .strafeToLinearHeading(smallTrianglePose)
                .build(),
            robot.shootBalls(rpmFar, 0),

            robot.drive.actionBuilder(smallTrianglePose)
                .strafeToLinearHeading(endPose)
                .build()
        )

        waitForStart()

        val action = when (robot.camera.detectAprilTagCase()) {
            21 -> actionPPG
            22 -> actionGPP
            else -> actionPGP
        }

        val dash = FtcDashboard.getInstance()
        val c = Canvas()
        action.preview(c)

        var b = true
        while (b && opModeIsActive()) {
            timeKeep.resetDeltaTime()
            robot.transfer.update(timeKeep.deltaTime)
            robot.shooter.update(timeKeep.deltaTime)

            val p = TelemetryPacket()
            p.fieldOverlay().operations.addAll(c.operations)

            b = action.run(p)

            dash.sendTelemetryPacket(p)
        }
    }
}