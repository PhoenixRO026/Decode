package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.ftc.Encoder
import com.commonlibs.units.Duration
import com.commonlibs.units.rpm
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.library.controller.PIDController

class Shooter(
    val motorTop: DcMotorEx,
    val motorBottom: DcMotorEx,
    val encoder: Encoder,
    val voltageSensor: VoltageSensor
) {
    @Config
    data object ShooterConfig {
        @JvmField
        var pidController = PIDController(
            kP = 0.0,
            kI = 0.0,
            kD = 0.0,
            zeroTargetReset = false
        )
        @JvmField
        var kS = 0.0
        @JvmField
        var kV = 0.0
        @JvmField
        var TICKS_PER_REV = 28
    }

    private val ticksPerSec get() = encoder.getPositionAndVelocity().velocity

    val rpm get() = (ticksPerSec / ShooterConfig.TICKS_PER_REV * 60.0).rpm

    var targetRpm = 0.0.rpm

    private var _power
        get() = motorTop.power
        set(value) {
            motorTop.power = value
            motorBottom.power = value
        }

    val power get() = _power

    fun update(deltaTime: Duration) {
        val voltage = voltageSensor.voltage
        val pidPower = ShooterConfig.pidController
            .calculate(rpm.asRpm, targetRpm.asRpm, deltaTime)
        val feedforwardPower = ShooterConfig.kS + ShooterConfig.kV * targetRpm.asRpm

        _power = pidPower + feedforwardPower / voltage
    }
}