package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.VelConstraint
import com.commonlibs.units.Pose
import com.commonlibs.units.deg
import com.commonlibs.units.inch
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.LimeLightCore.AutoCase
import org.firstinspires.ftc.teamcode.robot.Spindexer

@Autonomous
class BigTriangleBlue : LinearOpMode() {
    val startPose = Pose(-61.5.inch, -38.inch, 0.0.deg)
    val smallTrianglePose = Pose(55.inch, -10.inch, -90.deg)
    val bigTrianglePose = Pose(-14.inch, -16.inch, -90.0.deg)

    val rightIntakePose = Pose(36.inch, -30.inch, -90.0.deg)
    val rightIntakePoseBack = Pose(36.inch, -47.inch, -90.0.deg)
    val middleIntakePose = Pose(10.inch, -30.inch, -90.0.deg)
    val middleIntakePoseBack = Pose(10.inch, -55.inch, -90.0.deg)
    val leftIntakePose = Pose(-12.inch, -30.inch, -90.0.deg)
    val leftIntakePoseBack = Pose(-12.inch, -47.inch, -90.0.deg)

    val endPose = Pose(58.inch, -28.inch, 180.0.deg)

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
                kinematics.WheelVelConstraint(20.0),
                AngularVelConstraint(Math.toRadians(180.0))
            )
        )

        fun buildBigTriangleAction(vararg shootPositions: Spindexer.TransferPos): SequentialAction {

            return SequentialAction(
                ParallelAction(
                    robot.drive.actionBuilder(startPose)
                        .setTangent(45.deg)
                        .strafeToLinearHeading(bigTrianglePose)
                        .build(),
                    robot.shooter.goToRpmAction(robot.shooter.rpmClose),
                    robot.transfer.goToPosAction(Spindexer.TransferPos.shoot0),
                    robot.shooter.turretToPosAction(robot.shooter.shootClosePos)
                ),

                robot.shootBalls(),

                ParallelAction(
                    robot.drive.actionBuilder(bigTrianglePose)
                        .setTangent(0.deg)
                        .splineToLinearHeading(middleIntakePose, -90.deg)
                        .build(),
                    robot.intake.startIntakeAction()
                ),

                ParallelAction(
                    robot.drive.actionBuilder(middleIntakePose)
                        .setTangent(-90.deg)
                        .lineToY(-55.inch, slowSpeed)
                        .build(),
                    robot.intakeBalls(shootPositions[0])
                ),

                ParallelAction(
                    robot.drive.actionBuilder(middleIntakePoseBack)
                        .setTangent(90.deg)
                        .splineToLinearHeading(bigTrianglePose, 180.deg)
                        .build(),
                    robot.shooter.turretToPosAction(robot.shooter.shootClosePos),
                    robot.shooter.goToRpmAction(robot.shooter.rpmClose)
                ),
                robot.shootBalls(),

                ParallelAction(
                    robot.drive.actionBuilder(bigTrianglePose)
                        .setTangent(0.deg)
                        .splineToLinearHeading(leftIntakePose, -90.deg)
                        .build(),
                    robot.intake.startIntakeAction()
                ),

                ParallelAction(
                    robot.drive.actionBuilder(leftIntakePose)
                        .setTangent(-90.0)
                        .lineToY(-47.inch, slowSpeed)
                        .build(),
                    robot.intakeBalls(shootPositions[1])
                ),

                ParallelAction(
                    robot.drive.actionBuilder(leftIntakePoseBack)
                        .setTangent(90.deg)
                        .lineToY(-16.inch)
                        .build(),
                    robot.shooter.turretToPosAction(robot.shooter.shootClosePos),
                    robot.shooter.goToRpmAction(robot.shooter.rpmClose)
                ),

                robot.shootBalls(),

                ParallelAction(
                    robot.drive.actionBuilder(bigTrianglePose)
                        .setTangent(0.deg)
                        .splineToLinearHeading(rightIntakePose, -45.deg)
                        .build(),
                    robot.intake.startIntakeAction()
                ),

                ParallelAction(
                    robot.drive.actionBuilder(rightIntakePose)
                        .setTangent(-90.deg)
                        .lineToY(-47.inch, slowSpeed)
                        .build(),
                    robot.intakeBalls(shootPositions[2])
                ),

                ParallelAction(
                    robot.drive.actionBuilder(rightIntakePoseBack)
                        .setTangent(90.deg)
                        .splineToLinearHeading(smallTrianglePose, 0.deg)
                        .build(),
                    robot.shooter.turretToPosAction(robot.shooter.shootClosePos),
                    robot.shooter.goToRpmAction(robot.shooter.rpmClose)
                ),

                robot.shootBalls()
            )
        }

        val actionPGP = buildBigTriangleAction(
            Spindexer.TransferPos.shoot1,
            Spindexer.TransferPos.shoot2,
            Spindexer.TransferPos.shoot0
        )


        val actionPPG = buildBigTriangleAction(
            Spindexer.TransferPos.shoot2,
            Spindexer.TransferPos.shoot1,
            Spindexer.TransferPos.shoot0
        )

        val actionGPP = buildBigTriangleAction(
            Spindexer.TransferPos.shoot0,
            Spindexer.TransferPos.shoot1,
            Spindexer.TransferPos.shoot2
        )
        while (opModeInInit()) {
            robot.limelight.updateCase()
            telemetry.addData("case id", robot.limelight.currentCase)
            telemetry.addData("turret angle", robot.shooter.tickToDeg(robot.shooter.turretPosition))
            telemetry.addData("turret target angle", robot.shooter.tickToDeg(robot.shooter.targetPos))
            telemetry.update()
            sleep(20)
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
            telemetry.addData("turret angle", robot.shooter.tickToDeg(robot.shooter.turretPosition))
            telemetry.addData("rpm", robot.shooter.rpm)
            telemetry.update()
        }
    }
}
