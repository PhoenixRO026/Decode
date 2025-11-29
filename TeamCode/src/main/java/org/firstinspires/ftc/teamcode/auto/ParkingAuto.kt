package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.SequentialAction
import com.commonlibs.units.Distance2d
import com.commonlibs.units.Pose
import com.commonlibs.units.SleepAction
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.commonlibs.units.inch
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.robot.Robot

@Autonomous
class ParkingAuto : LinearOpMode() {
    val startPose = Pose(14.cm, -63.inch, 90.deg)
    val preloadSpecimenPos = Pose(14.cm, -30.5.inch, 90.deg)
    val zerothSpecimenPos = Pose(2.inch, -30.5.inch, 90.deg)
    val firstSpecimenPos = Pose(0.inch, -30.5.inch, 90.deg)
    val secondSpecimenPos =Pose(-2.inch, -30.5.inch, 90.deg)
    val thirdSpecimenPos =Pose(-4.inch, -30.5.inch, 90.deg)

    val sample3Heading = Distance2d(68.inch, -25.inch)

    val sample1 = Pose(48.inch, -49.inch, 90.deg)
    val sample2 = Pose(58.5.inch, -49.inch, 90.deg)
    val sample3 = Distance2d(61.inch, -49.inch).headingTowards(sample3Heading)

    val zonePos = Distance2d(45.1.inch, -67.2.inch)
    val zonePoze3 = Distance2d(50.inch, -67.2.inch)

    val firstKickPos = Distance2d(30.inch, -50.inch).headingTowards(zonePos)
    val secondKickPos = Distance2d(34.inch, -50.inch).headingTowards(zonePos)
    val thirdKickPos = Distance2d(38.inch, -50.inch).headingTowards(zonePos)
    val takeSpecimenPos = Pose(40.inch, -58.5.inch, 90.deg)

    override fun runOpMode() {
        initMessage()

        val timeKeep = TimeKeep()
        val robot = Robot(hardwareMap, startPose)
        val drive = robot.drive

        val action = SequentialAction(
            RaceAction(
                drive.actionBuilder(startPose)
                    .lineToY(-10.0.cm)
                    .build(),
                SleepAction(0.2.s)
            )
        )

        action.preview(previewCanvas)

        readyMessage()

        while (opModeInInit()) {
            timeKeep.resetDeltaTime()
            sleep(50)
        }

        var running = true

        while (isStarted && !isStopRequested && running) {
            timeKeep.resetDeltaTime()

            running = runAction(action)
            telemetry.update()
        }
    }

    private val previewCanvas = Canvas()

    private fun runAction(action: Action): Boolean {
        val packet = TelemetryPacket()
        packet.fieldOverlay().operations.addAll(previewCanvas.operations)

        val running = action.run(packet)

        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        return running
    }

    private fun initMessage() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.addLine("INITIALIZING")

        telemetry.update()
    }

    private fun readyMessage() {
        telemetry.addLine("READY")
        telemetry.update()
    }
}