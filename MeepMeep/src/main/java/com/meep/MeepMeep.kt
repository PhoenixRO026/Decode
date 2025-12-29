@file:JvmName("MeepMeep")
package com.meep

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

    val startPose = Pose(63.inch, -11.inch, 180.0.deg)
    val smallTrianglePose = Pose(57.inch, -10.inch, 210.0.deg)
    val bigTrianglePose = Pose(-14.inch, -14.inch, 225.0.deg)
    val rightIntakePose = Pose(36.inch, -35.inch, 270.0.deg)
    val middleIntakePose = Pose(12.inch, -35.inch, 270.0.deg)
    val leftIntakePose = Pose(-12.inch, -35.inch, 270.0.deg)
    val endPose = Pose(16.inch, -54.inch, 0.0.deg)

    val meepMeep = MeepMeep(600)

    val myBot =
        DefaultBotBuilder(meepMeep)
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .setDimensions(16.5,16.5)
            .setStartPose(startPose.pose2d)
            .build()


    myBot.runAction(myBot.drive.actionBuilder(startPose.pose2d).ex()
        .splineToLinearHeading(smallTrianglePose, 50.deg)
        .waitSeconds(1.0)

        .strafeToLinearHeading(rightIntakePose)
        .waitSeconds(1.0)
        .lineToY(-40.inch)
        .waitSeconds(1.0)
        .lineToY(-45.inch)
        .strafeToLinearHeading(smallTrianglePose)
        .waitSeconds(1.0)

        .strafeToLinearHeading(middleIntakePose)
        .setTangent(-90.deg)
        .lineToY(-40.inch)
        .waitSeconds(1.0)
        .lineToY(-45.inch)
        .strafeToLinearHeading(bigTrianglePose)

        .strafeToLinearHeading(leftIntakePose)
        .setTangent(-90.deg)
        .lineToY(-40.inch)
        .waitSeconds(1.0)
        .lineToY(-45.inch)
        .strafeToLinearHeading(bigTrianglePose)

        .strafeToLinearHeading(endPose)

        .build()
    )

    meepMeep.setBackground(Background.FIELD_DECODE_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}