package com.commonlibs

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.commonlibs.units.rotate

val turretOffsetFromRobot = Vector2d(-60.569 / 25.4, 0.0)

fun robotWorldPosToTurretWorldPos(robotPos: Pose2d): Vector2d {
    return robotPos * turretOffsetFromRobot
}

fun robotWorldvelToTurretWorldVel(robotVel: PoseVelocity2d, robotHeading: Rotation2d): Vector2d {
    return robotHeading * Vector2d(0.0, robotVel.angVel * turretOffsetFromRobot.x) + robotVel.linearVel
}