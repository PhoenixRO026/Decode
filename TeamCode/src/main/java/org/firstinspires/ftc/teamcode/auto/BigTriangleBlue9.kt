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
class BigTriangleBlue9 : LinearOpMode() {
    val startPose = Pose(-61.5.inch, -36.inch, 90.0.deg)
    val bigTrianglePose = Pose(-17.inch, -17.inch, 228.0.deg)

    val leftIntakePose = Pose(-11.5.inch, -28.inch, -90.0.deg)
    val middleIntakePose = Pose(10.inch, -27.inch, -90.0.deg)
    val leftIntakePoseBack = Pose(-11.5.inch, -28.inch, -90.0.deg)
    val middleIntakePoseBack = Pose(9.inch, -56.inch, -90.0.deg)

    val openGatePose = Pose(7.inch, -57.inch, -90.0.deg)

    val endPose = Pose(0.inch, -28.inch, -90.0.deg)

    val rpmFar = 3260.0
    val rpmClose = 2830.0

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

        val actionPGP = SequentialAction(
            ParallelAction (
                robot.shooter.goToRpmAction(rpmClose),
                robot.drive.actionBuilder(startPose)
                    .setTangent(45.0.deg)
                    .splineToLinearHeading(bigTrianglePose, -45.0.deg)
                    .build(),
                robot.transfer.goToPosAction(RobotConfig.pos, 0, RobotConfig.shooterOffset)
            ),

            robot.drive.correctionAction(bigTrianglePose, 1.0.s),

            robot.shootBalls(rpmClose, 0),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .setTangent(45.deg)
                .splineToLinearHeading(middleIntakePose, -45.deg)
                .build(),
            robot.drive.correctionAction(middleIntakePose, 0.75.s),

            RaceAction(
                robot.drive.actionBuilder(middleIntakePose)
                    .setTangent(-90.deg)
                    .lineToY(-54.inch, slowSpeed)
                    .strafeToLinearHeading(openGatePose)
                    .build(),
                robot.intakeBalls(1)
            ),
            ParallelAction(
                robot.intake.stopIntakeAction(),
                robot.shooter.goToRpmAction(rpmClose),
            ),

            robot.drive.actionBuilder(openGatePose)
                .setTangent(90.0.deg)
                .splineToLinearHeading(bigTrianglePose, -180.deg)
                .build(),
            robot.drive.correctionAction(bigTrianglePose, 1.0.s),

            robot.shootBalls(rpmClose, 1),


            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .setTangent(60.0.deg)
                .splineToLinearHeading(leftIntakePose, -90.deg)
                .build(),
            robot.drive.correctionAction(leftIntakePose, 0.75.s),

            RaceAction(
                robot.drive.actionBuilder(leftIntakePose)
                    .setTangent(-90.deg)
                    .lineToY(-48.inch, slowSpeed)
                    .build(),
                robot.intakeBalls(2)
            ),
            ParallelAction(
                robot.intake.stopIntakeAction(),
                robot.shooter.goToRpmAction(rpmClose),
            ),
            robot.drive.actionBuilder(leftIntakePoseBack)
                .setTangent(90.deg)
                .splineToLinearHeading(bigTrianglePose, -90.deg)
                .build(),
            robot.drive.correctionAction(bigTrianglePose, 1.5.s),
            robot.shootBalls(rpmClose, 2),

            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(endPose)
                .build(),
        )

        val actionPPG = SequentialAction(
            ParallelAction (
                robot.shooter.goToRpmAction(rpmClose),
                robot.drive.actionBuilder(startPose)
                    .setTangent(45.0.deg)
                    .splineToLinearHeading(bigTrianglePose, -45.0.deg)
                    .build(),
                robot.transfer.goToPosAction(RobotConfig.pos, 0, RobotConfig.shooterOffset)
            ),

            robot.drive.correctionAction(bigTrianglePose, 1.0.s),

            robot.shootBalls(rpmClose, 0),

            ParallelAction(
                robot.intake.startIntakeAction(),
                robot.drive.actionBuilder(bigTrianglePose)
                    .setTangent(45.deg)
                    .splineToLinearHeading(middleIntakePose, -45.deg)
                    .build(),
            ),

            robot.drive.correctionAction(middleIntakePose, 0.5.s),

            RaceAction(
                robot.drive.actionBuilder(middleIntakePose)
                    .setTangent(-90.deg)
                    .lineToY(-55.inch, slowSpeed)
                    .strafeToLinearHeading(openGatePose)
                    .build(),
                robot.intakeBalls(0)
            ),
            ParallelAction(
                robot.intake.stopIntakeAction(),
                robot.shooter.goToRpmAction(rpmClose),
                robot.drive.actionBuilder(openGatePose)
                    .setTangent(90.0.deg)
                    .splineToLinearHeading(bigTrianglePose, -180.deg)
                    .build(),
            ),

            robot.drive.correctionAction(bigTrianglePose, 1.0.s),

            robot.shootBalls(rpmClose, 0),

            ParallelAction(
                robot.intake.startIntakeAction(),
                robot.drive.actionBuilder(bigTrianglePose)
                    .setTangent(60.0.deg)
                    .splineToLinearHeading(leftIntakePose, -90.deg)
                    .build(),
            ),

            robot.drive.correctionAction(leftIntakePose, 0.75.s),

            RaceAction(
                robot.drive.actionBuilder(leftIntakePose)
                    .setTangent(-90.deg)
                    .lineToY(-48.inch, slowSpeed)
                    .build(),
                robot.intakeBalls(1)
            ),
            ParallelAction(
                robot.intake.stopIntakeAction(),
                robot.shooter.goToRpmAction(rpmClose),
                robot.drive.actionBuilder(leftIntakePoseBack)
                    .setTangent(90.deg)
                    .splineToLinearHeading(bigTrianglePose, -90.deg)
                    .build(),
            ),

            robot.drive.correctionAction(bigTrianglePose, 1.5.s),
            robot.shootBalls(rpmClose, 1),

            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(endPose)
                .build(),
        )

        val actionGPP = SequentialAction(
            ParallelAction (
                robot.shooter.goToRpmAction(rpmClose),
            robot.drive.actionBuilder(startPose)
                    .setTangent(45.0.deg)
                    .splineToLinearHeading(bigTrianglePose, -45.0.deg)
                    .build(),
                robot.transfer.goToPosAction(RobotConfig.pos, 0, RobotConfig.shooterOffset)
            ),
            robot.drive.correctionAction(bigTrianglePose, 1.0.s),

            robot.shootBalls(rpmClose, 0),

            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .setTangent(45.deg)
                .splineToLinearHeading(middleIntakePose, -45.deg)
                .build(),
            robot.drive.correctionAction(middleIntakePose, 0.75.s),

            RaceAction(
                robot.drive.actionBuilder(middleIntakePose)
                    .setTangent(-90.deg)
                    .lineToY(-54.inch, slowSpeed)
                    .strafeToLinearHeading(openGatePose)
                    .build(),
                robot.intakeBalls(2)
            ),
            ParallelAction(
                robot.intake.stopIntakeAction(),
                robot.shooter.goToRpmAction(rpmClose),
            ),

            robot.drive.actionBuilder(openGatePose)
                .setTangent(90.0.deg)
                .splineToLinearHeading(bigTrianglePose, -180.deg)
                .build(),
            robot.drive.correctionAction(bigTrianglePose, 1.0.s),

            robot.shootBalls(rpmClose, 2),


            robot.intake.startIntakeAction(),
            robot.drive.actionBuilder(bigTrianglePose)
                .setTangent(60.0.deg)
                .splineToLinearHeading(leftIntakePose, -90.deg)
                .build(),
            robot.drive.correctionAction(leftIntakePose, 0.75.s),

            RaceAction(
                robot.drive.actionBuilder(leftIntakePose)
                    .setTangent(-90.deg)
                    .lineToY(-48.inch, slowSpeed)
                    .build(),
                robot.intakeBalls(1)
            ),
            ParallelAction(
                robot.intake.stopIntakeAction(),
                robot.shooter.goToRpmAction(rpmClose),
            ),
            robot.drive.actionBuilder(leftIntakePoseBack)
                .setTangent(90.deg)
                .splineToLinearHeading(bigTrianglePose, -90.deg)
                .build(),
            robot.drive.correctionAction(bigTrianglePose, 1.5.s),
            robot.shootBalls(rpmClose, 1),

            robot.drive.actionBuilder(bigTrianglePose)
                .strafeToLinearHeading(endPose)
                .build(),

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
            telemetry.addData("color", robot.camera.colorSensor.getAnalysis())
            telemetry.addData("rpm", robot.shooter.rpm)
            telemetry.update()
        }
    }
}