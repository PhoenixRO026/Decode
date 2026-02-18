package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor

class VoltageKeep(hardwareMap: HardwareMap) : () -> Double {
    private val voltageSensor = hardwareMap.get(VoltageSensor::class.java, "Control Hub")
    private var _voltage = voltageSensor.voltage
    val voltage by ::_voltage

    fun update() {
        _voltage = voltageSensor.voltage
    }

    override fun invoke(): Double {
        return voltage
    }
}