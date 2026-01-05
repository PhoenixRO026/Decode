package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
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
class SmallTriangleRed : LinearOpMode() {

    val startPose = Pose(63.inch, 11.inch, (-180.0).deg)
    val smallTrianglePose = Pose(57.inch, 10.inch, (-200.0).deg)
    val bigTrianglePose = Pose(-10.inch, 10.inch, (-225.0).deg)

    val rightIntakePose = Pose(36.inch, 30.inch, (-270.0).deg)
    val middleIntakePose = Pose(12.inch, 30.inch, (-270.0).deg)
    val leftIntakePose = Pose(-12.inch, 30.inch, (-270.0).deg)

    val endPose = Pose(16.inch, 54.inch, 0.0.deg)

    val rpmFar = 3260.0
    val rpmClose = 2490.0

    val shooterOffset = 94.0

    var ticksPerRev = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0)
    var pos = ticksPerRev / 3.0

    override fun runOpMode() {

        val robot = Robot(hardwareMap, startPose)
        val timeKeep = TimeKeep()

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

        fun shoot(rpm : Double) = SequentialAction(
            robot.shooter.goToRpmAction(rpm),

            robot.transfer.goToPosAction(pos, 1, shooterOffset),
            robot.transfer.shootAction(),

            robot.transfer.goToPosAction(pos, 2, shooterOffset),
            robot.transfer.shootAction(),

            robot.transfer.goToPosAction(pos, 3, shooterOffset),
            robot.transfer.shootAction(),

            robot.shooter.goToRpmAction(0.0)
        )

        fun getBall() = SequentialAction (
            robot.intake.startIntakeAction(),
            robot.transfer.goToPosAction(pos, 3, 0.0),
            SleepAction(0.25.s),
            robot.transfer.goToPosAction(pos, 2, 0.0),
            SleepAction(0.25.s),
            robot.transfer.goToPosAction(pos, 1, 0.0),
            SleepAction(0.25.s)
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
                .setTangent(-90.deg)
                .afterTime(0.s, getBall())
                .lineToY(42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmFar))
                .strafeToLinearHeading(smallTrianglePose)
                .build(),

            robot.shootBalls(rpmFar, 0),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(smallTrianglePose)
                .strafeToLinearHeading(middleIntakePose)
                .setTangent(-90.deg)
                .afterTime(0.s, getBall())
                .lineToY(42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmClose))
                .strafeToLinearHeading(bigTrianglePose)
                .build(),

            robot.shootBalls(rpmClose, 1),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(leftIntakePose)
                .setTangent(-90.deg)
                .afterTime(0.s, getBall())
                .lineToY(42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .strafeToLinearHeading(bigTrianglePose)
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmClose))
                .build(),

            robot.shootBalls(rpmClose, 2),

            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(endPose)
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
                .setTangent(-90.deg)
                .afterTime(0.s, getBall())
                .lineToY(42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmFar))
                .strafeToLinearHeading(smallTrianglePose)
                .build(),

            robot.shootBalls(rpmFar, 2),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(smallTrianglePose)
                .strafeToLinearHeading(middleIntakePose)
                .setTangent(-90.deg)
                .afterTime(0.s, getBall())
                .lineToY(42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmClose))
                .strafeToLinearHeading(bigTrianglePose)
                .build(),

            robot.shootBalls(rpmClose, 0),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(leftIntakePose)
                .setTangent(-90.deg)
                .afterTime(0.s, getBall())
                .lineToY(42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .strafeToLinearHeading(bigTrianglePose)
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmClose))
                .build(),

            robot.shootBalls(rpmClose, 1),

            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(endPose)
                .build()
        )

        val actionGPP = SequentialAction(
            robot.drive.actionBuilder(startPose)
                .strafeToLinearHeading(smallTrianglePose)
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmFar))
                .build(),
            robot.drive.correctionAction(smallTrianglePose, 0.75.s),

            robot.shootBalls(rpmFar, 1),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(smallTrianglePose)
                .strafeToLinearHeading(rightIntakePose)
                .setTangent(-90.deg)
                .afterTime(0.s, getBall())
                .lineToY(42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmFar))
                .strafeToLinearHeading(smallTrianglePose)
                .build(),

            robot.shootBalls(rpmFar, 1),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(smallTrianglePose)
                .strafeToLinearHeading(middleIntakePose)
                .setTangent(-90.deg)
                .afterTime(0.s, getBall())
                .lineToY(42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmClose))
                .strafeToLinearHeading(bigTrianglePose)
                .build(),

            robot.shootBalls(rpmClose, 2),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(leftIntakePose)
                .setTangent(-90.deg)
                .afterTime(0.s, getBall())
                .lineToY(42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .strafeToLinearHeading(bigTrianglePose)
                .afterTime(0.0.s, robot.shooter.goToRpmAction(rpmClose))
                .build(),

            robot.shootBalls(rpmClose, 0),

            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(endPose)
                .build()
        )

        waitForStart()

        val action = when (robot.camera.detectAprilTagCase()) {
            21 -> actionGPP
            22 -> actionPGP
            else -> actionPPG
        }

        val dash = FtcDashboard.getInstance()
        val canvas = Canvas()
        action.preview(canvas)

        var running = true

        while (running && opModeIsActive()) {
            timeKeep.resetDeltaTime()
            robot.transfer.update(timeKeep.deltaTime)
            robot.shooter.update(timeKeep.deltaTime)

            val packet = TelemetryPacket()
            packet.fieldOverlay().operations.addAll(canvas.operations)

            running = action.run(packet)

            dash.sendTelemetryPacket(packet)

            telemetry.addData("case id", robot.camera.detectAprilTagCase())
            telemetry.addData("rpm", robot.shooter.rpm)
            telemetry.update()
        }
    }
}
