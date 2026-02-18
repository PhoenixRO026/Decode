package org.firstinspires.ftc.teamcode.robot

import com.commonlibs.units.Duration
import com.commonlibs.units.Pose
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.commonlibs.units.s
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.TimeKeep

class Robot(hardwareMap: HardwareMap, pose: Pose = Pose(0.cm, 0.cm, 0.deg)) {
    val timeKeep = TimeKeep()
    val voltageKeep = VoltageKeep(hardwareMap)
    val drive = Drive(hardwareMap, pose, voltageKeep)
    val shooter = Shooter(hardwareMap, voltageKeep)
    val spindexer = Spindexer(hardwareMap)
    val intake = Intake(hardwareMap)

    fun update() {
        timeKeep.resetDeltaTime()
        voltageKeep.update()
        drive.updatePoseEstimate()
        shooter.update(timeKeep.deltaTime)
        spindexer.update(timeKeep.deltaTime)
    }

    fun init() {
        spindexer.init()
    }

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addLine("==== PERFORMANCE ====")
        telemetry.addData("delta time", timeKeep.deltaTime)
        telemetry.addData("fps", 1.s / timeKeep.deltaTime)
        shooter.addTelemetry(telemetry)
        spindexer.addTelemetry(telemetry)
        intake.addTelemetry(telemetry)
    }
}