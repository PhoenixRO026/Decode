package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ftc.Encoder
import com.commonlibs.units.Duration
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import kotlin.math.abs

class Shooter(
    val motorTop: DcMotorEx,
    val motorBottom: DcMotorEx,
    val encoder: Encoder,
    val voltageSensor: VoltageSensor
)
{
    @Config
    data object ShooterConfig {
        @JvmField
        var controller = PIDController(
            kP = 0.002,
            kD = 0.00004,
            kI = 0.018,
            stabilityThreshold = 50.0
        )
        @JvmField var targetRpmTolerance = 50

        @JvmField
        var kS = 0.8
        @JvmField
        var kV = 0.002146

        var targetRpmTolerence = 100.0
    }

    val rpm get() = encoder.getPositionAndVelocity().velocity / 28.0 * 60

    var targetRpm = 0.0

    var _power
        get() = motorTop.power
        set(value) {
            motorTop.power = value
            motorBottom.power = value
        }


    var power
        get() = _power
        set(value) {
            if (targetRpm == 0.0)
                _power = 0.0
            else {
                _power = value.coerceIn(-1.0, 1.0)
            }
        }

    fun goToRmp(rpm : Double) {
        targetRpm = rpm
    }

    fun goToRpmAction(rpm: Double) = object : Action {
        var init = true
        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                goToRmp(rpm)
            }
            return abs(targetRpm - rpm) > ShooterConfig.targetRpmTolerance
        }
    }

    fun update(deltaTime: Duration) {
        val voltage = voltageSensor.voltage

        val pidPower = ShooterConfig.controller.calculate(rpm, targetRpm, deltaTime)

        val feedforwardPower = ShooterConfig.kS + ShooterConfig.kV * targetRpm

        power = pidPower + feedforwardPower / voltage
    }


    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("Outtake power", power)

        //telemetry.addData("lift current", rightMotor.getCurrent(CurrentUnit.AMPS) + leftMotor.getCurrent(CurrentUnit.AMPS))
    }
}