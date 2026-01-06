//BlueBigTriangleStart meepmeep
@file:JvmName("MeepMeep")
package com.meep

import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.VelConstraint
import com.commonlibs.roadrunnerext.ex
import com.commonlibs.units.Distance2d
import com.commonlibs.units.Pose
import com.commonlibs.units.deg
import com.commonlibs.units.inch
import com.commonlibs.units.s
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder

fun main() {
    System.setProperty("sun.java2d.opengl", "true")

    val startPose = Pose(-62.inch, -38.inch, 90.0.deg)
    val smallTrianglePose = Pose(55.inch, -10.inch, 202.0.deg)
    val bigTrianglePose = Pose(-19.inch, -19.inch, 225.0.deg)

    val rightIntakePose = Pose(36.inch, -32.inch, 270.0.deg)
    val middleIntakePose = Pose(14.inch, -33.inch, 270.0.deg)
    val leftIntakePose = Pose(-12.inch, -31.inch, 270.0.deg)
    val openGatePose = Pose(-3.inch, -55.inch, 180.0.deg)

    val endPose = Pose(10.inch, -58.inch, 220.0.deg)
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

        .strafeToLinearHeading(leftIntakePose)
        .setTangent(-90.deg)
        .lineToY(-45.inch, slowSpeed)
        .setTangent(0.0.deg)
        .splineToLinearHeading(openGatePose, 270.deg)
        .strafeToLinearHeading(bigTrianglePose)
        .waitSeconds(shootingTime)

        .strafeToLinearHeading(middleIntakePose)
        .setTangent(-90.deg)
        .lineToY(-45.inch, slowSpeed)
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