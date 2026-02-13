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
import com.commonlibs.units.SleepAction
import com.commonlibs.units.deg
import com.commonlibs.units.inch
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.robot.LimeLightCore
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.LimeLightCore.AutoCase
import org.firstinspires.ftc.teamcode.robot.Robot.RobotConfig

@Autonomous
class SmallTriangleBlueNew9 : LinearOpMode() {
    val startPose = Pose(61.inch, -11.inch, 180.0.deg)
    val smallTrianglePose = Pose(55.inch, -10.inch, 205.0.deg)
    val bigTrianglePose = Pose(-16.5.inch, -16.inch, 221.0.deg)

    val rightIntakePose = Pose(37.inch, -24.inch, -90.0.deg)
    val rightIntakePoseBack = Pose(37.inch, -47.inch, -90.0.deg)
    val middleIntakePose = Pose(12.inch, -28.inch, -90.0.deg)
    val middleIntakePoseBack = Pose(12.inch, -45.inch, -90.0.deg)
    val leftIntakePose = Pose(-11.5.inch, -28.inch, -90.0.deg)
    val humanIntakePose = Pose(54.inch, -58.inch, -45.0.deg)
    val humanIntakePoseBack = Pose(63.inch, -58.5.inch, -100.0.deg)

    val endPose = Pose(-1.inch, -25.inch, -90.0.deg)

    val rpmFar = 3280.0 // la rosu e 3260, again, trebe verificat
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


        /////////////////////////////////////////////////////

        fun buildSmallTriangleAction(vararg shootPositions: Int): SequentialAction {
            return SequentialAction(
                robot.shooter.goToRpmAction(rpmFar),
                ParallelAction(
                    robot.drive.actionBuilder(startPose)
                        .setTangent(180.0.deg)
                        .splineToLinearHeading(smallTrianglePose, 10.0.deg)
                        .build(),
                    robot.transfer.goToPosAction(RobotConfig.pos, 0, RobotConfig.shooterOffset)
                ),

                robot.drive.correctionAction(smallTrianglePose, 0.01.s),

                robot.shootBalls(rpmFar, 0),

                ParallelAction(
                    robot.intake.startIntakeAction(),
                    robot.drive.actionBuilder(smallTrianglePose)
                        .setTangent(-90.deg)
                        .splineToLinearHeading(humanIntakePose, 135.deg)
                        .build(),
                ),
                ParallelAction(
                    SequentialAction(
                        robot.drive.actionBuilder(humanIntakePose)
                        .setTangent(20.deg)
                        .strafeToLinearHeading(humanIntakePoseBack)
                        .build(),
                        robot.drive.correctionAction(humanIntakePoseBack, 1.s)
                    ),
                    robot.intakeBalls(shootPositions[0])
                ),
                ParallelAction(
                    robot.intake.stopIntakeAction(),
                    robot.shooter.goToRpmAction(rpmFar),
                    robot.drive.actionBuilder(humanIntakePoseBack)
                        .setTangent(90.0.deg)
                        .splineToLinearHeading(smallTrianglePose, -135.deg)
                        .build(),
                ),

                robot.drive.correctionAction(smallTrianglePose, 0.75.s),


                robot.shootBalls(rpmClose, shootPositions[0]),

                ///

                ParallelAction(
                    robot.intake.startIntakeAction(),
                    robot.drive.actionBuilder(smallTrianglePose)
                        .setTangent(0.deg)
                        .splineToLinearHeading(rightIntakePose, 45.deg)
                        .build(),
                ),

                robot.drive.correctionAction(rightIntakePose, 0.1.s),

                ParallelAction(
                    robot.drive.actionBuilder(rightIntakePose)
                        .setTangent(90.deg)
                        .lineToY(-47.inch, slowSpeed)
                        .build(),
                    robot.intakeBalls(shootPositions[2])
                ),

                ParallelAction(
                    robot.intake.stopIntakeAction(),
                    robot.shooter.goToRpmAction(rpmFar),
                    robot.drive.actionBuilder(rightIntakePoseBack)
                        .setTangent(90.0.deg)
                        .splineToLinearHeading(smallTrianglePose, -135.deg)
                        .build(),
                ),

                robot.drive.correctionAction(smallTrianglePose, 0.05.s),

                robot.shootBalls(rpmFar, shootPositions[2]),

            )
        }

        val actionPGP = buildSmallTriangleAction(
            1,
            2,
            0
        )


        val actionPPG = buildSmallTriangleAction(
            0,
            1,
            2
        )


        val actionGPP = buildSmallTriangleAction(
            2,
            0,
            1
        )


        val startAction =  InstantAction {
            robot.drive.actionBuilder(startPose)
                .strafeToLinearHeading(smallTrianglePose)}

        while (opModeInInit()) {
            robot.limelight.updateCase()
            telemetry.addData("case id", robot.limelight.currentCase)
            telemetry.update()
            sleep(20)
        }

        val action = SequentialAction(
            startAction,
            InstantAction{robot.limelight.updateCase()},
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

        telemetry.addData("True case: ",robot.limelight.updateCase())

        while (running && opModeIsActive()) {
            timeKeep.resetDeltaTime()
            robot.transfer.update(timeKeep.deltaTime)
            robot.shooter.update(timeKeep.deltaTime)

            val packet = TelemetryPacket()
            packet.fieldOverlay().operations.addAll(c.operations)


            running = action.run(packet)

            dash.sendTelemetryPacket(packet)
8
            telemetry.addData("case id", robot.limelight.updateCase())
            telemetry.addData("color", robot.camera.colorSensor.getAnalysis())
            telemetry.addData("rpm", robot.shooter.rpm)
            telemetry.update()
        }
    }
}