@file:JvmName("MeepMeep")
package com.meep

import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.VelConstraint
import com.commonlibs.roadrunnerext.ex
import com.commonlibs.units.Distance2d
import com.commonlibs.units.Pose
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.commonlibs.units.inch
import com.commonlibs.units.s
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder

data object blueGoal{
    val startPoseClose = Pose(63.inch, -11.inch, 180.0.deg)
    val startPoseFar = Pose(62.inch, -15.inch, -90.0.deg)
    val smallTrianglePose = Pose(55.inch, -14.inch, -90.0.deg)
    val bigTrianglePose = Pose(-16.inch, -16.inch, -90.0.deg) //trebe verficat?

    val rightIntakePose = Pose(36.inch, -30.inch, -90.0.deg)
    val middleIntakePose = Pose(14.inch, -30.inch, -90.0.deg) //trebe modificat
    val leftIntakePose = Pose(-12.inch, -30.inch, -90.0.deg)


    val endPose = Pose(58.inch, -28.inch, 180.0.deg)
}

data object blueGoal2{
    val startPose = Pose(-61.5.inch, -38.inch, -90.0.deg)
    val smallTrianglePose = Pose(55.inch, -10.inch, -90.deg)
    val bigTrianglePose = Pose(-14.inch, -16.inch, -90.0.deg)

    val rightIntakePose = Pose(36.inch, -30.inch, -90.0.deg)
    val rightIntakePoseBack = Pose(36.inch, -47.inch, -90.0.deg)
    val middleIntakePose = Pose(10.inch, -30.inch, -90.0.deg)
    val middleIntakePoseBack = Pose(10.inch, -58.inch, -90.0.deg)
    val leftIntakePose = Pose(-12.inch, -30.inch, -90.0.deg)
    val leftIntakePoseBack = Pose(-12.inch, -47.inch, -90.0.deg)


    val endPose = Pose(58.inch, -28.inch, 180.0.deg)
}

fun main() {
    System.setProperty("sun.java2d.opengl", "true")

    val shootingTime = 4.2

    val meepMeep = MeepMeep(600)

    val kinematics = MecanumKinematics(
        15.0,
        1.0
    )

    val slowSpeed: VelConstraint = MinVelConstraint(
        listOf(
            kinematics.WheelVelConstraint(10.0),
            AngularVelConstraint(Math.toRadians(180.0))
        )
    )

    val blueBot =
        DefaultBotBuilder(meepMeep)
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .setDimensions(16.5, 16.5)
            .setStartPose(blueGoal.startPoseClose.pose2d)
            .build()

    val blueBotClose =
        DefaultBotBuilder(meepMeep)
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .setDimensions(16.5, 16.5)
            .setStartPose(blueGoal.startPoseClose.pose2d)
            .build()

    val blueBotFar =
        DefaultBotBuilder(meepMeep)
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .setDimensions(16.5, 16.5)
            .setStartPose(blueGoal.startPoseFar.pose2d)
            .build()

    blueBotClose.runAction(blueBotClose.drive.actionBuilder(blueGoal2.startPose.pose2d).ex()
        .strafeToLinearHeading(blueGoal2.bigTrianglePose)

        .setTangent(0.deg)
        .splineToLinearHeading(blueGoal2.middleIntakePose, -90.deg)

        .setTangent(-90.deg)
        .lineToY(-58.inch, slowSpeed)

        .setTangent(90.deg)
        .splineToLinearHeading(blueGoal2.bigTrianglePose, 180.deg)

        .setTangent(-90.deg)
        .lineToY(-30.inch)
        .lineToY(-47.inch, slowSpeed)

        .setTangent(90.deg)
        .lineToY(-16.inch)

        .setTangent(0.deg)
        .splineToLinearHeading(blueGoal2.rightIntakePose, -45.deg)
        .setTangent(-90.deg)
        .lineToY(-47.inch, slowSpeed)

        .setTangent(90.deg)
        .splineToLinearHeading(blueGoal2.smallTrianglePose, 0.deg)


        .build()
    )

    blueBotFar.runAction(blueBotFar.drive.actionBuilder(blueGoal.startPoseFar.pose2d).ex()
        .strafeToLinearHeading(blueGoal.smallTrianglePose)

        .setTangent(0.deg)
        .strafeToLinearHeading(blueGoal.rightIntakePose)

        .setTangent(-90.deg)
        .lineToY(-45.inch, slowSpeed)

        .strafeToLinearHeading(blueGoal.smallTrianglePose)

        .strafeToLinearHeading(blueGoal.middleIntakePose)
        .setTangent(-90.deg)
        .lineToY(-45.inch, slowSpeed)
        .strafeToLinearHeading(blueGoal.smallTrianglePose)

        .build()
    )


    meepMeep.setBackground(Background.FIELD_DECODE_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(blueBotClose)
        //.addEntity(blueBotFar)
        .start()
}