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
    //val startPose = Pose(23.0.inch, -60.0.inch, 90.0.deg)

    val startPose = Pose(-38.5.inch, -62.inch, 90.0.deg)
    val pivot = Pose(-47.0.inch, -47.0.inch, 90.deg)
    val basket = Pose(-53.5.inch, -53.5.inch, 45.0.deg)
    val first_yellow = Distance2d(-55.5.inch, -46.5.inch).headingTowards(Distance2d(-47.inch, -26.0.inch))
    val mid_yellow = Distance2d(-58.0.inch, -45.0.inch).headingTowards(Distance2d(-58.inch, -26.0.inch))
    val last_yellow = Distance2d(-57.0.inch, -43.0.inch).headingTowards(Distance2d(-69.inch, -26.0.inch))

    /*
    * 48 26
    * 58 26
    * 69 26
    * */


    val meepMeep = MeepMeep(600)

    val myBot =
        DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .setDimensions(15.748,15.748)
            .build()

    myBot.runAction(myBot.drive.actionBuilder(startPose.pose2d).ex()
        /*.setTangent(90.0.deg)
        .strafeToLinearHeading(basket.position, basket.heading)

        .setTangent(90.0.deg)
        .strafeToLinearHeading(first_yellow.position, first_yellow.heading)

        .waitSeconds(3.s)
        .setTangent(-90.0.deg)
        .strafeToLinearHeading(basket.position, basket.heading)


        .waitSeconds(3.s)
        .setTangent(45.0.deg)
        .strafeToLinearHeading(mid_yellow.position, mid_yellow.heading)


        .waitSeconds(2.5.s)
        .setTangent(-90.0.deg)
        .strafeToLinearHeading(basket.position, basket.heading)

        .waitSeconds(6.s)
        .setTangent(0.0.deg)
        .strafeToLinearHeading(last_yellow.position, last_yellow.heading)


        .waitSeconds(2.5.s)
        .setTangent(-90.0.deg)
        .strafeToLinearHeading(basket.position, basket.heading)

        .waitSeconds(6.s)*/

        .setTangent(90.0.deg)
        .strafeToLinearHeading(basket.position, basket.heading)
        .setTangent(90.0.deg)
        .strafeToLinearHeading(first_yellow.position, first_yellow.heading)
        .setTangent(-90.0.deg)
        .strafeToLinearHeading(basket.position, basket.heading)
        .setTangent(45.0.deg)
        .strafeToLinearHeading(mid_yellow.position, mid_yellow.heading)
        .setTangent(-90.0.deg)
        .strafeToLinearHeading(basket.position, basket.heading)
        .setTangent(0.0.deg)
        .strafeToLinearHeading(last_yellow.position, last_yellow.heading)
        .waitSeconds(2.5.s)
        .setTangent(-90.0.deg)

        .build()
    )

    meepMeep.setBackground(Background.FIELD_DECODE_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}