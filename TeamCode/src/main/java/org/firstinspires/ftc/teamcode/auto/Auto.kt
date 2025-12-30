package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.commonlibs.units.Pose
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.commonlibs.units.inch
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.Robot

@Autonomous
class Auto : LinearOpMode() {
    val startPose = Pose(63.inch, -11.inch, 180.0.deg)
    val smallTrianglePose = Pose(57.inch, -10.inch, 210.0.deg)
    val bigTrianglePose = Pose(-14.inch, -14.inch, 225.0.deg)
    val rightIntakePose = Pose(36.inch, -35.inch, 270.0.deg)
    val middleIntakePose = Pose(12.inch, -35.inch, 270.0.deg)
    val leftIntakePose = Pose(-12.inch, -35.inch, 270.0.deg)
    val endPose = Pose(16.inch, -54.inch, 0.0.deg)
    val shootingTime = 4.2

    val rpmFar = 3300
    val rpmClose = 2500

    var ticksPerRev = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0)
    var pos = ticksPerRev / 3.0
    var multiplier = 0.0

    override fun runOpMode() {
        val robot = Robot(hardwareMap, startPose)

        val shoot = SequentialAction(
            robot.transfer.spinAction()
        )

        val action = SequentialAction(
            robot.drive.actionBuilder(startPose)
                .splineToLinearHeading(smallTrianglePose, 50.deg)
                .afterTime(0.0.s, shoot)
                .waitSeconds(shootingTime)
                .strafeToLinearHeading(rightIntakePose)
                .waitSeconds(0.5)
                .lineToY(-40.inch)
                .waitSeconds(0.5)
                .lineToY(-45.inch)
                .strafeToLinearHeading(smallTrianglePose)
                .waitSeconds(shootingTime)

                .strafeToLinearHeading(middleIntakePose)
                .setTangent(-90.deg)
                .lineToY(-40.inch)
                .waitSeconds(0.5)
                .lineToY(-45.inch)
                .strafeToLinearHeading(bigTrianglePose)
                .waitSeconds(shootingTime)

                .strafeToLinearHeading(leftIntakePose)
                .setTangent(-90.deg)
                .lineToY(-40.inch)
                .waitSeconds(0.5)
                .lineToY(-45.inch)
                .strafeToLinearHeading(bigTrianglePose)
                .waitSeconds(shootingTime)

                .strafeToLinearHeading(endPose)

                .build()

        )

        waitForStart()

        runBlocking(action)
    }
}