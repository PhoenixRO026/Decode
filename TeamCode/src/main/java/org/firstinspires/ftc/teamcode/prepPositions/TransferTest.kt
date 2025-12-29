package org.firstinspires.ftc.teamcode.prepPositions

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.commonlibs.units.Pose
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.robot_old.Robot

@TeleOp
class TransferTest : LinearOpMode(){
    @Config
    data object TransferTestConfig {
        @JvmField var fingerPos = 1.0
        @JvmField var pos = 0.0
        @JvmField var multiplier = 1.0
        @JvmField var offset = 0.0
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))
        val timeKeep = TimeKeep()

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            robot.transfer.finger.position = TransferTestConfig.fingerPos
            robot.transfer.goToPos(TransferTestConfig.pos, TransferTestConfig.multiplier, TransferTestConfig.offset)

            robot.transfer.update(timeKeep.deltaTime)

            telemetry.addData("transfer pos", robot.transfer.position)
            telemetry.addData("transfer target pos", robot.transfer.targetPosition)
            telemetry.addData("power trans", robot.transfer.power)

            telemetry.addData("fingir", robot.transfer.finger.position)

            telemetry.addData("delta time ms", timeKeep.deltaTime.asMs)
            telemetry.addData("fps", 1.s / timeKeep.deltaTime)
            telemetry.update()
        }
    }
}