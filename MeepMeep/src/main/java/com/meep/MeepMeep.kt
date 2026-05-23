@file:JvmName("MeepMeep")
package com.meep

import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.VelConstraint
import com.commonlibs.roadrunnerext.ex
import com.commonlibs.robotWorldPosToTurretWorldPos
import com.commonlibs.robotWorldvelToTurretWorldVel
import com.commonlibs.units.Pose
import com.commonlibs.units.deg
import com.commonlibs.units.inch
import com.commonlibs.units.rotate
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder

data object blueGoalFarSolo{
    val startPoseClose = Pose(63.inch, -11.inch, 180.0.deg)
    val startPoseFar = Pose(62.inch, -15.inch, -90.0.deg)
    val smallTrianglePose = Pose(55.inch, -14.inch, -90.0.deg)
    val bigTrianglePose = Pose(-16.inch, -16.inch, -90.0.deg)

    val rightIntakePose = Pose(36.inch, -30.inch, -90.0.deg)
    val middleIntakePose = Pose(14.inch, -30.inch, -90.0.deg)
    val leftIntakePose = Pose(-12.inch, -30.inch, -90.0.deg)

    val humanIntakePose = Pose(55.inch, -59.inch, 300.0.deg)
    val humanIntakePoseBack = Pose(60.inch, -59.inch, 290.0.deg)

    val endPose = Pose(58.inch, -30.inch, -90.0.deg)
}

data object blueGoalFarDuo{
    val startPoseClose = Pose(63.inch, -11.inch, 180.0.deg)
    val startPoseFar = Pose(62.inch, -15.inch, -90.0.deg)
    val smallTrianglePose = Pose(55.inch, -14.inch, -90.0.deg)
    val bigTrianglePose = Pose(-16.inch, -16.inch, -90.0.deg)

    val rightIntakePose = Pose(36.inch, -30.inch, -90.0.deg)
    val firstCycle = Pose(37.inch, -58.inch, -160.0.deg)
    val firstCycleBack = Pose(23.inch, -58.inch, 180.0.deg)

    val humanIntakePose = Pose(56.inch, -55.inch, -70.0.deg)
    val humanGetReady = Pose(54.inch, -57.inch, -40.0.deg)
    val humanIntakePoseBack = Pose(56.inch, -60.inch, 0.0.deg)

    val endPose = Pose(58.inch, -30.inch, -90.0.deg)
}

data object blueGoalCloseSolo{
    val startPose = Pose(-61.5.inch, -38.inch, -90.0.deg)
    val smallTrianglePose = Pose(55.inch, -10.inch, -90.deg)
    val bigTrianglePose = Pose(-14.inch, -16.inch, -90.0.deg)

    val rightIntakePose = Pose(36.inch, -30.inch, -90.0.deg)
    val rightIntakePoseBack = Pose(36.inch, -47.inch, -90.0.deg)
    val middleIntakePose = Pose(10.inch, -30.inch, -90.0.deg)
    val middleIntakePoseBack = Pose(10.inch, -58.inch, -90.0.deg)
    val leftIntakePose = Pose(-12.inch, -30.inch, -90.0.deg)
    val leftIntakePoseBack = Pose(-12.inch, -47.inch, -90.0.deg)


    val endPose = Pose(58.inch, -30.inch, 180.0.deg)
}

data object blueGoalCloseDuo{
    val startPose = Pose(-47.5.inch, -52.5.inch, -125.0.deg)
    val bigTrianglePreloadPose = Pose(2.inch, -11.inch, -125.deg)
    val bigTriangleParkPose = Pose(-25.inch, -11.5.inch, -135.deg)
    val bigTrianglePose = Pose(4.inch, -9.inch, -90.0.deg)

    val rightIntakePose = Pose(36.inch, -29.inch, -90.0.deg)
    val middleIntakePose = Pose(14.inch, -32.inch, -90.0.deg)
    val middleIntakePoseBack = Pose(14.inch, -50.inch, -90.0.deg)
    val leftIntakePose = Pose(-10.inch, -30.inch, -90.0.deg)
    val leftIntakePoseBack = Pose(-11.inch, -52.inch, -90.0.deg)

    val openGatePose = Pose(11.inch, -56.5.inch, -100.deg)
    val openGatePoseBack = Pose(21.5.inch, -59.inch, -110.deg)
    val nextToGatePose = Pose(17.inch, -60.inch, -115.deg)
}

data object redGoalCloseDuo{
    val startPose = Pose(61.5.inch, -14.inch, -90.0.deg)
    val shootPose = Pose(61.5.inch, -28.inch, -90.0.deg)
    val intakePose = Pose(61.5.inch, -57.inch, -90.0.deg)

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
            kinematics.WheelVelConstraint(30.0),
            AngularVelConstraint(Math.toRadians(180.0))
        )
    )


    val blueBotCloseSolo =
        DefaultBotBuilder(meepMeep)
            .setConstraints(80.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .setDimensions(16.9, 16.9)
            .setStartPose(blueGoalFarSolo.startPoseClose.pose2d)
            .build()

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    val blueBotCloseDuo =
        DefaultBotBuilder(meepMeep)
            .setConstraints(80.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .setDimensions(16.9, 16.9)
            .setStartPose(blueGoalFarSolo.startPoseClose.pose2d)
            .build()

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    val blueBotFarSolo =
        DefaultBotBuilder(meepMeep)
            .setConstraints(80.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .setDimensions(16.9, 16.9)
            .setStartPose(blueGoalFarSolo.startPoseFar.pose2d)
            .build()

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    val blueBotFarDuo =
        DefaultBotBuilder(meepMeep)
            .setConstraints(80.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .setDimensions(16.9, 16.9)
            .setStartPose(blueGoalFarSolo.startPoseFar.pose2d)
            .build()

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    val redBotCloseDuo =
        DefaultBotBuilder(meepMeep)
            .setConstraints(80.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .setDimensions(16.9, 16.9)
            .setStartPose(redGoalCloseDuo.startPose.pose2d)
            .build()

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    redBotCloseDuo.runAction(redBotCloseDuo.drive.actionBuilder(redGoalCloseDuo.startPose.pose2d).ex()
        .setTangent(-90.deg)
        .lineToY(redGoalCloseDuo.intakePose.position.y)
        .setTangent(90.deg)
        .lineToY(redGoalCloseDuo.shootPose.position.y)

        .setTangent(-90.deg)
        .lineToY(redGoalCloseDuo.intakePose.position.y)
        .setTangent(90.deg)
        .lineToY(redGoalCloseDuo.shootPose.position.y)

        .setTangent(-90.deg)
        .lineToY(redGoalCloseDuo.intakePose.position.y)
        .setTangent(90.deg)
        .lineToY(redGoalCloseDuo.shootPose.position.y)

        .build()
    )

    blueBotCloseSolo.runAction(blueBotCloseSolo.drive.actionBuilder(blueGoalCloseSolo.startPose.pose2d).ex()
        .strafeToLinearHeading(blueGoalCloseDuo.bigTrianglePreloadPose)

        .setTangent(0.deg)
        .splineToLinearHeading(blueGoalCloseDuo.middleIntakePose, -90.deg)

        .setTangent(-90.deg)
        .lineToY(-58.inch, slowSpeed)

        .setTangent(90.deg)
        .splineToLinearHeading(blueGoalCloseSolo.bigTrianglePose, 180.deg)

        .setTangent(-90.deg)
        .lineToY(-30.inch)
        .lineToY(-47.inch, slowSpeed)

        .setTangent(90.deg)
        .lineToY(-16.inch)

        .setTangent(0.deg)
        .splineToLinearHeading(blueGoalCloseSolo.rightIntakePose, -45.deg)
        .setTangent(-90.deg)
        .lineToY(-47.inch, slowSpeed)

        .setTangent(90.deg)
        .splineToLinearHeading(blueGoalCloseSolo.smallTrianglePose, 0.deg)


        .build()
    )

    blueBotCloseDuo.runAction(blueBotCloseDuo.drive.actionBuilder(blueGoalCloseDuo.startPose.pose2d).ex()
        .strafeToLinearHeading(blueGoalCloseDuo.bigTrianglePreloadPose)

        .setTangent(-30.deg)
        .splineToLinearHeading(blueGoalCloseDuo.middleIntakePose, -90.deg)

        .setTangent(90.deg)
        .lineToY(-50.inch, slowSpeed)

        .setTangent(90.deg)
        .splineToLinearHeading(blueGoalCloseDuo.bigTrianglePose, 90.deg)

        .setTangent(-90.deg)
        .splineToLinearHeading(blueGoalCloseDuo.openGatePose, -90.deg)

        .setTangent(90.deg)
        .splineToLinearHeading(blueGoalCloseDuo.bigTrianglePose, 90.deg)

        .setTangent(-90.deg)
        .splineToLinearHeading(blueGoalCloseDuo.openGatePose, -90.deg)

        .setTangent(90.deg)
        .splineToLinearHeading(blueGoalCloseDuo.bigTrianglePose, 90.deg)

        .setTangent(-90.deg)
        .splineToLinearHeading(blueGoalCloseDuo.leftIntakePose, -120.deg)
        .setTangent(-90.deg)
        .lineToY(-47.inch, slowSpeed)

        .setTangent(90.deg)
        .splineToLinearHeading(blueGoalCloseDuo.bigTriangleParkPose, 90.deg)

        .build()
    )

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    blueBotFarSolo.runAction(blueBotFarSolo.drive.actionBuilder(blueGoalFarSolo.startPoseFar.pose2d).ex()
        .strafeToLinearHeading(blueGoalFarSolo.smallTrianglePose)

        .strafeToLinearHeading(blueGoalFarSolo.humanIntakePose)
        .strafeToLinearHeading(blueGoalFarSolo.humanIntakePoseBack)
        .strafeToLinearHeading(blueGoalFarSolo.endPose)
        .splineToLinearHeading(blueGoalFarSolo.smallTrianglePose, 70.deg)
        .splineToLinearHeading(blueGoalFarSolo.smallTrianglePose, 70.deg)

        .setTangent(0.deg)
        .strafeToLinearHeading(blueGoalFarSolo.rightIntakePose)

        .setTangent(-90.deg)
        .lineToY(-45.inch, slowSpeed)

        .strafeToLinearHeading(blueGoalFarSolo.smallTrianglePose)
        .build()
    )

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    blueBotFarDuo.runAction(blueBotFarDuo.drive.actionBuilder(blueGoalFarDuo.startPoseClose.pose2d).ex()
        .strafeToLinearHeading(blueGoalFarDuo.smallTrianglePose)

        .setTangent(0.deg)
        .strafeToLinearHeading(blueGoalFarDuo.rightIntakePose)

        .setTangent(-90.deg)
        .lineToY(-47.inch, slowSpeed)

        .setTangent(90.deg)
        .strafeToLinearHeading(blueGoalFarDuo.smallTrianglePose)

        .setTangent(-90.deg)
        .strafeToLinearHeading(blueGoalFarDuo.humanIntakePose)

        .setTangent(90.deg)
        .strafeToLinearHeading(blueGoalFarDuo.humanGetReady)

        .setTangent(180.deg)
        .strafeToLinearHeading(blueGoalFarDuo.humanIntakePoseBack)

        .setTangent(30.deg)
        .strafeToLinearHeading(blueGoalFarDuo.smallTrianglePose)

        .setTangent(-90.deg)
        .splineToLinearHeading(blueGoalFarDuo.firstCycle, -135.0.deg)
        .setTangent(180.0.deg)
        .splineToLinearHeading(blueGoalFarDuo.firstCycleBack, 0.0.deg)

        .setTangent(90.deg)
        .strafeToLinearHeading(blueGoalFarDuo.smallTrianglePose)

        .strafeToLinearHeading(blueGoalFarDuo.endPose)

        .build()
    )

    meepMeep.setBackground(Background.FIELD_DECODE_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(blueBotCloseDuo)
        //.addEntity(blueBotFar)
        .start()
}