package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.VelConstraint
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.commonlibs.units.Pose
import com.commonlibs.units.SleepAction
import com.commonlibs.units.deg
import com.commonlibs.units.inch
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.robot.Robot

/*
@Autonomous
class TestingAuto : LinearOpMode() {
    val startPose = Pose(63.inch, -11.inch, 180.0.deg)
    val smallTrianglePose = Pose(57.inch, -10.inch, 210.0.deg)
    val bigTrianglePose = Pose(-14.inch, -14.inch, 225.0.deg)
    val rightIntakePose = Pose(36.inch, -35.inch, 270.0.deg)
    val middleIntakePose = Pose(12.inch, -35.inch, 270.0.deg)
    val leftIntakePose = Pose(-12.inch, -35.inch, 270.0.deg)
    val endPose = Pose(16.inch, -54.inch, 0.0.deg)
    val shootingTime = 4.2

    val rpmFar = 3300.0
    val rpmClose = 2500.0

    val shooterOffset = 94.0

    var ticksPerRev = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0)
    var pos = ticksPerRev / 3.0

    override fun runOpMode() {
        val timeKeep = TimeKeep()
        val robot = Robot(hardwareMap, startPose)

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

        fun shoot() = SequentialAction(
            robot.shooter.goToRpmAction(rpmFar),
            robot.transfer.goToPosAction(pos, 1.0, 0.0),
            robot.transfer.shootAction(),

            robot.transfer.goToPosAction(pos, 2.0, 0.0),
            robot.transfer.shootAction(),

            robot.transfer.goToPosAction(pos, 3.0, 0.0),
            robot.transfer.shootAction(),

            robot.shooter.goToRpmAction(0.0)
        )

        fun getBall() = SequentialAction (
            robot.intake.startIntakeAction(),
            robot.transfer.goToPosAction(pos, 3.0, 0.0),
            robot.transfer.goToPosAction(pos, 2.0, 0.0),
            robot.transfer.goToPosAction(pos, 1.0, 0.0),
            robot.intake.stopIntakeAction()
        )

        val action = SequentialAction(
            SequentialAction(
                robot.transfer.goToPosAction(pos, 3.0, 0.0),
                SleepAction(0.25.s),
                robot.transfer.goToPosAction(pos, 2.0, 0.0),
                SleepAction(0.25.s),
                robot.transfer.goToPosAction(pos, 1.0, 0.0),
                SleepAction(0.25.s)
            ),
            robot.drive.actionBuilder(startPose)

                .build()
        )

        waitForStart()

        val dash = FtcDashboard.getInstance()
        val c = Canvas()
        action.preview(c)

        var b = true
        while (b && opModeIsActive()) {
            timeKeep.resetDeltaTime()
            robot.transfer.update(timeKeep.deltaTime)

            val p = TelemetryPacket()
            p.fieldOverlay().operations.addAll(c.operations)

            b = action.run(p)

            dash.sendTelemetryPacket(p)
        }
    }
}*/
