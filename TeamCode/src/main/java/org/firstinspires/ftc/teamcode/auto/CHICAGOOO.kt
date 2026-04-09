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
class CHICAGOOO : LoggedOpMode() {
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
                kinematics.WheelVelConstraint(27.0),
                AngularVelConstraint(Math.toRadians(180.0))
            )
        )

        fun buildBigTriangleAction(vararg shootPositions: Spindexer.TransferPos): SequentialAction {
            return SequentialAction(
                ParallelAction(
                    robot.drive.actionBuilder(startPose)
                        .strafeToLinearHeading(bigTrianglePreloadPose)
                        .build(),
                    SequentialAction(
                        ParallelAction(
                            robot.shooter.goToRpmAction(robot.shooter.rpmClose + 75),
                            robot.transfer.goToPosAction(Spindexer.TransferPos.shoot0),
                            robot.shooter.turretToPosAction(robot.shooter.shootClosePos  - robot.shooter.degToTick(30.0)),
                        ),
                        robot.shootBalls(robot.shooter.rpmClose)
                    )
                ),

                ParallelAction(
                    robot.drive.actionBuilder(bigTrianglePreloadPose)
                        .setTangent(-30.deg)
                        .splineToLinearHeading(middleIntakePose, -90.deg)
                        .lineToY(-56.inch)
                        .setTangent(90.deg)
                        .splineToLinearHeading(bigTrianglePose, 90.deg)
                        .build(),
                    SequentialAction(
                        ParallelAction(
                            robot.shooter.goToRpmAction(robot.shooter.rpmRest),
                            robot.intakeBalls(shootPositions[0])
                        ),
                        ParallelAction(
                            robot.shooter.turretToPosAction(robot.shooter.shootClosePos),
                            robot.shooter.goToRpmAction(robot.shooter.rpmClose)
                        ),
                        robot.shootBalls(robot.shooter.rpmClose),
                    )
                ),


                ParallelAction(
                    robot.drive.actionBuilder(bigTrianglePose)
                        .setTangent(-90.0.deg)
                        .splineToLinearHeading(openGatePose, -90.deg)
                        .setTangent(-35.deg)
                        .splineToLinearHeading(openGatePoseBack, -90.deg)
                        .turnTo(-90.deg)
                        .build(),
                    robot.intakeBalls(shootPositions[2]),
                    robot.shooter.goToRpmAction(robot.shooter.rpmRest),
                    robot.shooter.turretToPosAction(robot.shooter.shootClosePos),
                ),

                ParallelAction(
                    robot.drive.actionBuilder(openGatePoseBack)
                        .setTangent(90.deg)
                        .splineToLinearHeading(bigTrianglePose, 90.deg)
                        .build(),
                    robot.shooter.goToRpmAction(robot.shooter.rpmClose)
                ),

                robot.shootBalls(robot.shooter.rpmClose),


                ParallelAction(
                    robot.drive.actionBuilder(bigTrianglePose)
                        .setTangent(0.0.deg)
                        .splineToLinearHeading(rightIntakePose, -45.deg)
                        .setTangent(-90.deg)
                        .lineToY(-56.inch)
                        .setTangent(135.deg)
                        .splineToLinearHeading(bigTrianglePose, 180.deg)
                        .build(),
                    SequentialAction(
                        ParallelAction(
                            robot.intakeBalls(shootPositions[1]),
                            robot.shooter.goToRpmAction(robot.shooter.rpmRest)
                        ),
                        ParallelAction(
                            robot.shooter.turretToPosAction(robot.shooter.shootClosePos),
                            robot.shooter.goToRpmAction(robot.shooter.rpmClose)
                        ),
                    )
                ),
                robot.shootBalls(robot.shooter.rpmClose),


                ParallelAction(
                    robot.drive.actionBuilder(bigTrianglePose)
                        .setTangent(-90.0.deg)
                        .splineToLinearHeading(leftIntakePose, -120.deg)
                        .setTangent(-90.deg)
                        .lineToY(-52.inch)
                        .setTangent(90.deg)
                        .lineToY(-23.inch)
                        .build(),
                    robot.intakeBalls(shootPositions[3]),
                    robot.shooter.turretToPosAction(robot.shooter.shootClosePos),
                    robot.shooter.goToRpmAction(robot.shooter.rpmClose - 100)
                ),

                robot.shootBalls(robot.shooter.rpmClose),
                robot.drive.actionBuilder(bigTrianglePose)
                    .setTangent(-90.0.deg)
                    .lineToY(-20.0.inch)
                    .build(),
            )

        }

        val actionPGP = buildBigTriangleAction(
            Spindexer.TransferPos.shoot2,
            Spindexer.TransferPos.shoot2,
            Spindexer.TransferPos.shoot0,
            Spindexer.TransferPos.shoot1
        )


        val actionPPG = buildBigTriangleAction(
            Spindexer.TransferPos.shoot1,
            Spindexer.TransferPos.shoot1,
            Spindexer.TransferPos.shoot2,
            Spindexer.TransferPos.shoot0
        )

        val actionGPP = buildBigTriangleAction(
            Spindexer.TransferPos.shoot0,
            Spindexer.TransferPos.shoot0,
            Spindexer.TransferPos.shoot1,
            Spindexer.TransferPos.shoot2
        )

        robot.transfer.goToPos(Spindexer.TransferPos.shoot0)

        while (opModeInInit()) {
            timeKeep.resetDeltaTime()
            robot.shooter.currentMode = MODE.PID
            robot.shooter.targetPos = Shooter.ShooterConfig.cameraPos
            robot.shooter.updateTurretPosition(timeKeep.deltaTime)
            robot.limelight.updateCase()
            telemetry.addData("case id", robot.limelight.currentCase)
            telemetry.update()
        }

        val action = when (robot.limelight.currentCase) {
            AutoCase.GPP -> actionGPP
            AutoCase.PGP -> actionPGP
            else -> actionPPG
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