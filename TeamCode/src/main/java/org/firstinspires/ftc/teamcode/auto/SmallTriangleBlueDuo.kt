package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.VelConstraint
import com.commonlibs.units.Pose
import com.commonlibs.units.deg
import com.commonlibs.units.inch
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.LoggedOpMode
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.LimeLightCore.AutoCase
import org.firstinspires.ftc.teamcode.robot.Spindexer

@Autonomous
class SmallTriangleBlueDuo : LinearOpMode() {
    val startPose = Pose(63.inch, -11.inch, 180.0.deg)
    val smallTrianglePose = Pose(55.inch, -14.inch, -90.0.deg)

    val rightIntakePose = Pose(36.inch, -30.inch, -90.0.deg)
    val rightIntakePoseBack = Pose(36.inch, -56.inch, -90.0.deg)
    val firstCycle = Pose(37.inch, -58.inch, -160.0.deg)
    val firstCycleBack = Pose(23.inch, -58.inch, 180.0.deg)

    val humanIntakePose = Pose(56.inch, -55.inch, -70.0.deg)
    val humanGetReady = Pose(54.inch, -57.inch, -40.0.deg)
    val humanIntakePoseBack = Pose(56.inch, -60.inch, 0.0.deg)

    val endPose = Pose(58.inch, -30.inch, -90.0.deg)
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
                        .strafeToLinearHeading(smallTrianglePose)
                        .build(),
                    robot.shooter.goToRpmAction(robot.shooter.rpmFar),
                    robot.transfer.goToPosAction(shootPositions[0]),
                    robot.shooter.turretToPosAction(robot.shooter.shootFarPos)
                ),

                robot.shootBallsRaw(robot.shooter.rpmFar),

                ParallelAction(
                    robot.drive.actionBuilder(smallTrianglePose)
                        .setTangent(0.deg)
                        .strafeToLinearHeading(rightIntakePose)
                        .build(),
                    robot.intake.startIntakeAction()
                ),

                ParallelAction(
                    robot.drive.actionBuilder(rightIntakePose)
                        .setTangent(-90.deg)
                        .lineToY(-56.inch, slowSpeed)
                        .build(),
                    robot.intakeBalls(shootPositions[1])
                ),

                ParallelAction(
                    robot.drive.actionBuilder(rightIntakePoseBack)
                        .setTangent(60.deg)
                        .strafeToLinearHeading(smallTrianglePose)
                        .build(),
                    robot.shooter.turretToPosAction(robot.shooter.shootFarPos),
                    robot.shooter.goToRpmAction(robot.shooter.rpmFar)
                ),

                robot.shootBallsRaw(robot.shooter.rpmFar),

                ParallelAction(
                    robot.drive.actionBuilder(smallTrianglePose)
                        .setTangent(0.deg)
                        .strafeToLinearHeading(humanIntakePose)
                        .build(),
                    robot.intake.startIntakeAction()
                ),

                ParallelAction(
                    robot.drive.actionBuilder(humanIntakePose)
                        .setTangent(90.deg)
                        .strafeToLinearHeading(humanGetReady)
                        .setTangent(180.deg)
                        .strafeToLinearHeading(humanIntakePoseBack)
                        //.turnTo(-30.deg)
                        //.turnTo(0.deg)
                        .build(),
                    robot.intakeBalls(shootPositions[2])
                ),

                ParallelAction(
                    robot.drive.actionBuilder(humanIntakePoseBack)
                        .strafeToLinearHeading(smallTrianglePose)
                        .build(),
                    robot.shooter.turretToPosAction(robot.shooter.shootFarPos),
                    robot.shooter.goToRpmAction(robot.shooter.rpmFar)
                ),

                robot.shootBallsRaw(robot.shooter.rpmFar),

                ParallelAction(
                    robot.drive.actionBuilder(smallTrianglePose)
                        .setTangent(-90.deg)
                        .splineToLinearHeading(firstCycle, -135.0.deg)
                        .build(),
                    robot.intake.startIntakeAction()
                ),

                ParallelAction(
                    robot.drive.actionBuilder(firstCycle)
                        .setTangent(180.0.deg)
                        .splineToLinearHeading(firstCycleBack, 0.0.deg)
                        .build(),
                    robot.intakeBalls(shootPositions[1])
                ),

                ParallelAction(
                    robot.drive.actionBuilder(firstCycleBack)
                        .setTangent(60.deg)
                        .strafeToLinearHeading(smallTrianglePose)
                        .build(),
                    robot.shooter.turretToPosAction(robot.shooter.shootFarPos),
                    robot.shooter.goToRpmAction(robot.shooter.rpmFar)
                ),

                robot.shootBallsRaw(robot.shooter.rpmFar),

                robot.drive.actionBuilder(smallTrianglePose)
                    .setTangent(-90.deg)
                    .lineToY(-20.inch)
                    .build()
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

        robot.transfer.goToPos(Spindexer.TransferPos.shoot0)

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
            telemetry.addData("target rpm", robot.shooter.targetRpm)
            telemetry.addData("current color", robot.transfer.sensorColor)
            telemetry.update()
        }
    }
}
