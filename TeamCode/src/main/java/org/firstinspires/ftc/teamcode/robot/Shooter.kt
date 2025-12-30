package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.commonlibs.roadrunnerext.ActionWithInit
import com.commonlibs.units.AngularVelocity
import com.commonlibs.units.Duration
import com.commonlibs.units.rpm
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import kotlin.math.absoluteValue

class Shooter(
    val motorTop: DcMotorEx,
    val motorBottom: DcMotorEx,
    val encoder: Encoder,
    private val voltageProvider: () -> Double
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
        @JvmField
        var targetRpmTolerance = 30
    }

    constructor(hardwareMap: HardwareMap) : this(
        hardwareMap,
        object : () -> Double {
            val voltageSensor = hardwareMap.voltageSensor.iterator().next()
            override fun invoke(): Double {
                return voltageSensor.voltage
            }
        }
    )

    constructor(hardwareMap: HardwareMap, voltageProvider: () -> Double) : this(
        motorTop = hardwareMap.get(DcMotorEx::class.java, "motorShooterTop"),
        motorBottom = hardwareMap.get(DcMotorEx::class.java, "motorShooterBotton"),
        encoder = OverflowEncoder(RawEncoder(hardwareMap
            .get(DcMotorEx::class.java, "motorRB"))),
        voltageProvider
    ) {
        motorTop.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorTop.direction = DcMotorSimple.Direction.FORWARD
        motorTop.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        motorBottom.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorBottom.direction = DcMotorSimple.Direction.REVERSE
        motorBottom.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        encoder.direction = DcMotorSimple.Direction.REVERSE
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

    private var voltage = voltageProvider()

    val power get() = _power

    fun update(deltaTime: Duration) {
        voltage = voltageProvider()
        val pidPower = ShooterConfig.pidController
            .calculate(rpm.asRpm, targetRpm.asRpm, deltaTime)
        val feedforwardPower = ShooterConfig.kS + ShooterConfig.kV * targetRpm.asRpm

        _power = pidPower + feedforwardPower / voltage
    }

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addLine("===== Shooter =====")
        telemetry.addData("current rpm", rpm)
        telemetry.addData("target rpm", targetRpm)
        telemetry.addData("power", power)
        telemetry.addData("voltage", voltage)
    }

    //ACTIONS
    fun waitForRpmAction(targetRpm: AngularVelocity) = ActionWithInit(
        init = { this.targetRpm = targetRpm },
        run = {
            it.addLine("Waiting for rpm to reach target")
            (targetRpm - rpm).asRpm.absoluteValue > ShooterConfig.targetRpmTolerance
        }
    )
}