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
    Pose smallTrianglePose = Pose(57.inch, -10.inch, 210.0.deg)

    val meepMeep = MeepMeep(600)

    val myBot =
        DefaultBotBuilder(meepMeep)
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .setDimensions(15.748,15.748)
            .setStartPose(startPose.pose2d)
            .build()


    myBot.runAction(myBot.drive.actionBuilder(startPose.pose2d).ex()
        .lineToSpl
        .build()
    )

    meepMeep.setBackground(Background.FIELD_DECODE_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}