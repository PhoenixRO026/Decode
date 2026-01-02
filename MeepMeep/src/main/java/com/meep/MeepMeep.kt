//BlueBigTriangleStart meepmeep
@file:JvmName("MeepMeep")
package com.meep

import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.VelConstraint
import com.commonlibs.roadrunnerext.ex
import com.commonlibs.units.Pose
import com.commonlibs.units.deg
import com.commonlibs.units.inch
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder

fun main() {
    System.setProperty("sun.java2d.opengl", "true")

    val startPose = Pose(-50.inch, 50.7.inch, -54.0.deg)
    val smallTrianglePose = Pose(57.inch, -10.inch, 200.0.deg)
    val bigTrianglePose = Pose(-10.inch, 10.inch, 135.0.deg)

    val rightIntakePose = Pose(36.inch, -30.inch, 270.0.deg)
    val middleIntakePose = Pose(12.inch, 30.inch, 90.0.deg)
    val leftIntakePose = Pose(-12.inch, 30.inch, 90.0.deg)

    val endPose = Pose(10.inch, 58.inch, 160.0.deg)
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

    val myBot =
        DefaultBotBuilder(meepMeep)
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .setDimensions(16.5,16.5)
            .setStartPose(startPose.pose2d)
            .build()


    myBot.runAction(myBot.drive.actionBuilder(startPose.pose2d).ex()
        .strafeToLinearHeading(bigTrianglePose)
        .waitSeconds(shootingTime)

        .strafeToLinearHeading(middleIntakePose)
        .setTangent(-90.deg)
        .lineToY(45.inch, slowSpeed)
        .strafeToLinearHeading(bigTrianglePose)
        .waitSeconds(shootingTime)

        .strafeToLinearHeading(leftIntakePose)
        .setTangent(-90.deg)
        .lineToY(45.inch, slowSpeed)
        .strafeToLinearHeading(bigTrianglePose)
        .waitSeconds(shootingTime)

        .strafeToLinearHeading(endPose)

        .build()
    )

    meepMeep.setBackground(Background.FIELD_DECODE_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}