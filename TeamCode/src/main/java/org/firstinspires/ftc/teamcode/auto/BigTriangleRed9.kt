package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.VelConstraint
import com.commonlibs.units.Pose
import com.commonlibs.units.deg
import com.commonlibs.units.inch
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.LimeLightCore.AutoCase
import org.firstinspires.ftc.teamcode.robot.Robot.RobotConfig

@Autonomous
class BigTriangleRed9 : LinearOpMode() {
    val startPose = Pose(-61.5.inch, 36.inch, -90.0.deg)
    val bigTrianglePose = Pose(-16.inch, 24.inch, 140.0.deg)
    val smallTrianglePose = Pose(53.inch, 14.inch, 160.0.deg)


    val leftIntakePose = Pose(-11.5.inch, 25.inch, 90.0.deg)
    val middleIntakePose = Pose(9.5.inch, 27.inch, 90.0.deg)
    val leftIntakePoseBack = Pose(-11.5.inch, 46.inch, 90.0.deg)
    val middleIntakePoseBack = Pose(9.5.inch, 57.inch, 90.0.deg)
    val rightIntakePose = Pose(37.inch, 24.inch, 90.0.deg)
    val rightIntakePoseBack = Pose(37.inch, 46.inch, 90.0.deg)
    val openGatePose = Pose(6.inch, 57.inch, 90.0.deg)

    val endPose = Pose(0.inch, -28.inch, -90.0.deg)

    val rpmFar = 3260.0
    val rpmClose = 2850.0

    val shooterOffset = 94.0

    var ticksPerRev = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0)
    var pos = ticksPerRev / 3.0

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
                kinematics.WheelVelConstraint(11.0),
                AngularVelConstraint(Math.toRadians(180.0))
            )
        )

        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        fun buildBigTriangleAction(vararg shootPositions: Int): SequentialAction {
            return SequentialAction(
                robot.shooter.goToRpmAction(rpmClose),
                ParallelAction (
                    robot.drive.actionBuilder(startPose)
                        .setTangent(-30.0.deg)
                        .splineToLinearHeading(bigTrianglePose, -10.0.deg)
                        .build(),
                    robot.transfer.goToPosAction(RobotConfig.pos, 0, RobotConfig.shooterOffset)
                ),

                robot.drive.correctionAction(bigTrianglePose, 0.01.s),

                robot.shootBalls(rpmClose, 0),

                ParallelAction(
                    robot.intake.startIntakeAction(),
                    robot.drive.actionBuilder(bigTrianglePose)
                        .setTangent(0.deg)
                        .splineToLinearHeading(middleIntakePose, 45.deg)
                        .build(),
                ),

                robot.drive.correctionAction(middleIntakePose, 0.1.s),

                ParallelAction(
                    robot.drive.actionBuilder(middleIntakePose)
                        .setTangent(90.deg)
                        .lineToY(57.inch, slowSpeed)
                        .build(),
                    robot.intakeBalls(shootPositions[0])
                ),

                ParallelAction(
                    robot.intake.stopIntakeAction(),
                    robot.shooter.goToRpmAction(rpmClose),
                    robot.drive.actionBuilder(middleIntakePoseBack)
                        .setTangent(-90.0.deg)
                        .splineToLinearHeading(bigTrianglePose, 180.deg)
                        .build(),
                ),

                robot.drive.correctionAction(bigTrianglePose, 0.02.s),

                robot.shootBalls(rpmClose, shootPositions[0]),

                ParallelAction(
                    robot.intake.startIntakeAction(),
                    robot.drive.actionBuilder(bigTrianglePose)
                        .setTangent(-60.0.deg)
                        .splineToLinearHeading(leftIntakePose, 90.deg)
                        .build(),
                ),

                robot.drive.correctionAction(leftIntakePose, 0.1.s),

                ParallelAction(
                    robot.drive.actionBuilder(leftIntakePose)
                        .setTangent(90.deg)
                        .lineToY(46.inch, slowSpeed)
                        .build(),
                    robot.intakeBalls(shootPositions[1])
                ),
                ParallelAction(
                    robot.intake.stopIntakeAction(),
                    robot.shooter.goToRpmAction(rpmClose),
                    robot.drive.actionBuilder(leftIntakePoseBack)
                        .setTangent(-90.deg)
                        .splineToLinearHeading(bigTrianglePose, -90.deg)
                        .build(),
                ),

                robot.shootBalls(rpmClose, shootPositions[1]),

                ParallelAction(
                    robot.intake.startIntakeAction(),
                    robot.drive.actionBuilder(bigTrianglePose)
                        .setTangent(0.deg)
                        .lineToXLinearHeading(37.inch, 90.deg)
                        .build(),
                ),

                ParallelAction(
                    robot.drive.actionBuilder(rightIntakePose)
                        .setTangent(90.deg)
                        .lineToY(46.inch, slowSpeed)
                        .build(),
                    robot.intakeBalls(shootPositions[2])
                ),
                ParallelAction(
                    robot.intake.stopIntakeAction(),
                    robot.shooter.goToRpmAction(rpmFar),
                    robot.drive.actionBuilder(rightIntakePoseBack)
                        .setTangent(-90.0.deg)
                        .splineToLinearHeading(smallTrianglePose, -45.deg)
                        .build(),
                ),

                robot.drive.correctionAction(smallTrianglePose, 0.75.s),


                robot.shootBalls(rpmClose, shootPositions[2]),
            )
        }

        val actionPGP = buildBigTriangleAction(
            1,
            2,
            0
        )


        val actionPPG = buildBigTriangleAction(
            0,
            1,
            2
        )


        val actionGPP = buildBigTriangleAction(
            2,
            0,
            1
        )


        while (opModeInInit()) {
            robot.limelight.updateCase()
            telemetry.addData("case id", robot.limelight.currentCase)
            telemetry.update()
            sleep(20)
        }

        val action = SequentialAction(
            //startAction,
            when (robot.limelight.currentCase) {
                AutoCase.GPP -> actionGPP
                AutoCase.PGP -> actionPGP
                else -> actionPPG
            }
        )


        val dash = FtcDashboard.getInstance()
        val c = Canvas()
        action.preview(c)

        var running = true

        telemetry.addData("True case: ",robot.limelight.currentCase)

        while (running && opModeIsActive()) {
            timeKeep.resetDeltaTime()
            robot.transfer.update(timeKeep.deltaTime)
            robot.shooter.update(timeKeep.deltaTime)

            val packet = TelemetryPacket()
            packet.fieldOverlay().operations.addAll(c.operations)


            running = action.run(packet)

            dash.sendTelemetryPacket(packet)

            telemetry.addData("case id", robot.limelight.currentCase)
            telemetry.addData("color", robot.camera.sensorColor)
            telemetry.addData("rpm", robot.shooter.rpm)
            telemetry.update()
        }
    }
}