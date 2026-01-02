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
class SmallTriangleBlue : LinearOpMode() {
    val startPose = Pose(63.inch, -11.inch, 180.0.deg)
    val smallTrianglePose = Pose(55.inch, -10.inch, 200.0.deg)
    val bigTrianglePose = Pose(-10.inch, -10.inch, 220.0.deg)

    val rightIntakePose = Pose(36.inch, -30.inch, 270.0.deg)
    val middleIntakePose = Pose(12.inch, -30.inch, 270.0.deg)
    val leftIntakePose = Pose(-12.inch, -30.inch, 270.0.deg)


    val endPose = Pose(16.inch, -54.inch, 0.0.deg)

    val rpmFar = 3250.0
    val rpmClose = 2500.0

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
            robot.transfer.goToPosAction(pos, 1.0, shooterOffset),
            robot.transfer.shootAction(),

            robot.transfer.goToPosAction(pos, 2.0, shooterOffset),
            robot.transfer.shootAction(),

            robot.transfer.goToPosAction(pos, 3.0, shooterOffset),
            robot.transfer.shootAction(),
            ParallelAction(
                robot.shooter.goToRpmAction(0.0),
                robot.transfer.goToPosAction(pos, 3.0, 0.0)
            )
        )

        fun getBall() = SequentialAction (
            //robot.intake.startIntakeAction(),
            //robot.transfer.goToPosAction(pos, 3.0, 0.0),
            SleepAction(0.35.s),
            robot.transfer.goToPosAction(pos, 2.0, 0.0),
            SleepAction(0.4.s),
            robot.transfer.goToPosAction(pos, 1.0, 0.0),
            SleepAction(0.25.s)
        )

        val action = SequentialAction(
            robot.drive.actionBuilder(startPose)
                //.splineToLinearHeading(smallTrianglePose, 70.deg)
                .strafeToLinearHeading(smallTrianglePose)
                .build(),
            robot.drive.correctionAction(smallTrianglePose, 1.s),
            shoot(rpmFar),
            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(smallTrianglePose)
                .strafeToLinearHeading(rightIntakePose)
                .setTangent(90.deg)
                .afterTime(0.s, getBall())
                .lineToY(-42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .strafeToLinearHeading(smallTrianglePose)
                .build(),
            shoot(rpmFar),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(smallTrianglePose)
                .strafeToLinearHeading(middleIntakePose)
                .setTangent(90.deg)
                .afterTime(0.s, getBall())
                .lineToY(-42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .strafeToLinearHeading(bigTrianglePose)
                .build(),
            shoot(rpmClose),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(leftIntakePose)
                .setTangent(90.deg)
                .afterTime(0.s, getBall())
                .lineToY(-42.inch, slowSpeed)
                .afterTime(0.0, robot.intake.stopIntakeAction())
                .strafeToLinearHeading(bigTrianglePose)
                .build(),
            shoot(rpmClose),

            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(endPose)
                .build()
        )

        waitForStart()

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