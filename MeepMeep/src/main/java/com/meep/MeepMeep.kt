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

data object redGoal{
    val startPose = Pose(63.inch, 11.inch, 180.0.deg)
    val smallTrianglePose = Pose(55.inch, 10.inch, 158.0.deg)
    val bigTrianglePose = Pose(-16.inch, 16.inch, 139.0.deg) //trebe verficat?

    val rightIntakePose = Pose(36.inch, 29.inch, 90.0.deg)
    val middleIntakePose = Pose(14.inch, 29.inch, 90.0.deg) //trebe modificat
    val leftIntakePose = Pose(-12.inch, 29.inch, 90.0.deg)

    val endPose = Pose(58.inch, 28.inch, 180.0.deg)
}

data object blueGoal{
    val startPose = Pose(63.inch, -11.inch, 180.0.deg)
    val smallTrianglePose = Pose(55.inch, -10.inch, 202.0.deg)
    val bigTrianglePose = Pose(-16.inch, -16.inch, 221.0.deg) //trebe verficat?

    val rightIntakePose = Pose(36.inch, -30.inch, -90.0.deg)
    val middleIntakePose = Pose(14.inch, -30.inch, -90.0.deg) //trebe modificat
    val leftIntakePose = Pose(-12.inch, -30.inch, -90.0.deg)

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

    val redBot =
        DefaultBotBuilder(meepMeep)
            .setConstraints(60.0, 100.0, Math.toRadians(180.0) , Math.toRadians(180.0), 15.0)
            .setDimensions(16.5, 16.5)
            .setStartPose(redGoal.startPose.pose2d)
            .build()

    val blueBot =
        DefaultBotBuilder(meepMeep)
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .setDimensions(16.5, 16.5)
            .setStartPose(blueGoal.startPose.pose2d)
            .build()


    redBot.runAction(redBot.drive.actionBuilder(redGoal.startPose.pose2d).ex()
        .strafeToLinearHeading(redGoal.smallTrianglePose)
        .waitSeconds(shootingTime)

        .strafeToLinearHeading(redGoal.rightIntakePose)
        .setTangent(90.deg)
        .lineToY(43.inch, slowSpeed)
        .setTangent(0.0.deg)
        .strafeToLinearHeading(redGoal.smallTrianglePose)

        .strafeToLinearHeading(redGoal.middleIntakePose)
        .setTangent(90.deg)
        .lineToY(43.inch, slowSpeed)
        .setTangent(-90.deg)
        .splineToLinearHeading(redGoal.bigTrianglePose, 180.deg)

        .strafeToLinearHeading(redGoal.leftIntakePose)

        .setTangent(90.deg)
        .lineToY(43.inch, slowSpeed)

        .build()
    )
    blueBot.runAction(blueBot.drive.actionBuilder(blueGoal.startPose.pose2d).ex()
        .strafeToLinearHeading(blueGoal.smallTrianglePose)
        .waitSeconds(shootingTime)

        .strafeToLinearHeading(blueGoal.rightIntakePose)
        .setTangent(-90.deg)
        .lineToY(-45.inch, slowSpeed)
        .setTangent(0.0.deg)
        .strafeToLinearHeading(blueGoal.smallTrianglePose)

        .strafeToLinearHeading(blueGoal.middleIntakePose)
        .setTangent(-90.deg)
        .lineToY(-45.inch, slowSpeed)
        .setTangent(90.deg)
        .splineToLinearHeading(blueGoal.bigTrianglePose, 180.deg)

        .strafeToLinearHeading(blueGoal.leftIntakePose)

        .setTangent(90.deg)
        .lineToY(-45.inch, slowSpeed)

        .build()
    )

    meepMeep.setBackground(Background.FIELD_DECODE_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        //.addEntity(blueBot)
        .addEntity(redBot)
        .start()
}