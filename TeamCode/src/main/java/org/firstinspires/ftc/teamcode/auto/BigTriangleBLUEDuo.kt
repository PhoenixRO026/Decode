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
class BigTriangleBLUEDuo : LinearOpMode() {
    val startPose = Pose(-61.5.inch, -38.inch, 90.0.deg)
    val bigTrianglePose = Pose(-5.inch, -16.inch, -90.0.deg)
    val middleIntakePose = Pose(12.inch, -28.inch, -90.0.deg)
    val middleIntakePoseBack = Pose(6.inch, -52.inch, -90.0.deg)
    val leftIntakePoseBack = Pose(-12.inch, -52.inch, -90.0.deg)

    val openGatePose = Pose(10.inch, -58.inch, -130.deg)


    val endPose = Pose(58.inch, -30.inch, 180.0.deg)

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
                    robot.intake.startIntakeAction(),
                ),

                ParallelAction(
                    robot.drive.actionBuilder(middleIntakePose)
                        .setTangent(-90.deg)
                        .lineToY(-47.inch, slowSpeed)
                        .build(),
                    robot.intakeBalls(shootPositions[0])
                ),

                // Y -47 -> bigTriangle
                ParallelAction(
                    robot.drive.actionBuilder(middleIntakePoseBack)
                        .setTangent(90.deg)
                        .splineToLinearHeading(bigTrianglePose, 180.deg)
                        .build(),
                    robot.shooter.turretToPosAction(robot.shooter.shootClosePos),
                    robot.shooter.goToRpmAction(robot.shooter.rpmClose)
                ),

                robot.shootBalls(),

                // bigTriangle -> openGate
                ParallelAction(
                    robot.drive.actionBuilder(bigTrianglePose)
                        .setTangent(0.deg)
                        .splineToLinearHeading(openGatePose, -90.deg)
                        .build(),
                    robot.intake.startIntakeAction()
                ),

                // openGate -> bigTriangle
                ParallelAction(
                    robot.drive.actionBuilder(openGatePose)
                        .setTangent(90.deg)
                        .splineToLinearHeading(bigTrianglePose, 180.deg)
                        .build(),
                    robot.intakeBalls(shootPositions[1]),
                    robot.shooter.turretToPosAction(robot.shooter.shootClosePos),
                    robot.shooter.goToRpmAction(robot.shooter.rpmClose)
                ),

                robot.shootBalls(),

                // bigTriangle -> openGate (again)
                ParallelAction(
                    robot.drive.actionBuilder(bigTrianglePose)
                        .setTangent(0.deg)
                        .splineToLinearHeading(openGatePose, -90.deg)
                        .build(),
                    robot.intake.startIntakeAction()
                ),

                // openGate -> bigTriangle (again)
                ParallelAction(
                    robot.drive.actionBuilder(openGatePose)
                        .setTangent(90.deg)
                        .splineToLinearHeading(bigTrianglePose, 180.deg)
                        .build(),
                    robot.intakeBalls(shootPositions[2]),
                    robot.shooter.turretToPosAction(robot.shooter.shootClosePos),
                    robot.shooter.goToRpmAction(robot.shooter.rpmClose)
                ),

                robot.shootBalls(),

                // bigTriangle -> Y -30 -> Y -47
                ParallelAction(
                    robot.drive.actionBuilder(bigTrianglePose)
                        .setTangent(-90.deg)
                        .lineToY(-30.inch)
                        .lineToY(-47.inch, slowSpeed)
                        .build()
                ),

                // Y -47 -> Y -16
                ParallelAction(
                    robot.drive.actionBuilder(leftIntakePoseBack)
                        .setTangent(90.deg)
                        .lineToY(-16.inch)
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
            telemetry.addData("rpm", robot.shooter.rpm)
            telemetry.update()
        }
    }
}
