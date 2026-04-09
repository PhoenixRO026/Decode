package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.VelConstraint
import com.commonlibs.units.Pose
import com.commonlibs.units.SleepAction
import com.commonlibs.units.deg
import com.commonlibs.units.inch
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.LoggedOpMode
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.LimeLightCore.AutoCase
import org.firstinspires.ftc.teamcode.robot.Shooter
import org.firstinspires.ftc.teamcode.robot.Shooter.MODE
import org.firstinspires.ftc.teamcode.robot.Spindexer

@Autonomous
class CHICAGOOOcycle : LoggedOpMode() {
    val startPose = Pose(61.5.inch, -14.inch, -90.0.deg)
    val shootPose = Pose(61.5.inch, -28.inch, -90.0.deg)
    val intakePose = Pose(61.5.inch, -58.inch, -90.0.deg)

    val shooterOffset = 94.0


    override fun runOpMode() {
        val robot = Robot(hardwareMap, startPose)
        val timeKeep = TimeKeep()

        robot.limelight.setPipeline(0)

        robot.transfer.fingerDown()

        val kinematics = MecanumKinematics(
            15.0,
            1.0
        )

        val slowSpeed: VelConstraint = MinVelConstraint(
            listOf(
                kinematics.WheelVelConstraint(37.0),
                AngularVelConstraint(Math.toRadians(180.0))
            )
        )

        fun makeCycle(): Action = SequentialAction(
            ParallelAction(
                robot.drive.actionBuilder(shootPose)
                    .setTangent(-90.deg)
                    .lineToY(intakePose.position.y, slowSpeed)
                    .build(),
                robot.intakeBallsCycle(Spindexer.TransferPos.shoot0)
            ),
            ParallelAction(
                robot.drive.actionBuilder(intakePose)
                    .setTangent(90.deg)
                    .lineToY(shootPose.position.y)
                    .build(),
                robot.shooter.turretToPosAction(robot.shooter.shootChicagoFarPos),
                robot.shooter.goToRpmAction(robot.shooter.rpmFar)
            ),
            ParallelAction(
                robot.shootBalls(robot.shooter.rpmFar),
                robot.drive.correctionAction(shootPose, 0.75.s)
            )
        )

        val actions = mutableListOf<Action>()

// first cycle if it is different, put it here
        actions += SequentialAction(
            ParallelAction(
                robot.drive.actionBuilder(startPose)
                    .setTangent(-90.deg)
                    .lineToY(intakePose.position.y,slowSpeed)
                    .build(),
                robot.intakeBallsCycle(Spindexer.TransferPos.shoot0)
            ),
            ParallelAction(
                robot.drive.actionBuilder(intakePose)
                    .setTangent(90.deg)
                    .lineToY(shootPose.position.y)
                    .build(),
                robot.shooter.turretToPosAction(robot.shooter.shootChicagoFarPos),
                robot.shooter.goToRpmAction(robot.shooter.rpmFar)
            ),
            ParallelAction(
                robot.shootBalls(robot.shooter.rpmFar),
                robot.drive.correctionAction(shootPose, 0.75.s)
            )
        )

        repeat(6) {
            actions += makeCycle()
        }

        val action = SequentialAction(*actions.toTypedArray())

        robot.transfer.goToPos(Spindexer.TransferPos.intake0)

        while (opModeInInit()) {
            robot.limelight.updateCase()
            telemetry.addData("case id", robot.limelight.currentCase)
            telemetry.update()
        }

        val dash = FtcDashboard.getInstance()
        val c = Canvas()
        action.preview(c)

        var running = true

        telemetry.addData("True case: ", robot.limelight.updateCase())

        while (running && opModeIsActive()) {
            timeKeep.resetDeltaTime()
            robot.shooter.updateRpm(timeKeep.deltaTime)
            robot.shooter.updateTurretPosition(timeKeep.deltaTime)


            val packet = TelemetryPacket()
            packet.fieldOverlay().operations.addAll(c.operations)

            running = action.run(packet)

            dash.sendTelemetryPacket(packet)
            telemetry.addData("rpm", robot.shooter.rpm)
            telemetry.update()
        }
    }
}