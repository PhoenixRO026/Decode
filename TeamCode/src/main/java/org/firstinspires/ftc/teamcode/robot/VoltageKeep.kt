package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.HardwareMap

class VoltageKeep(hardwareMap: HardwareMap) : () -> Double {
    private val voltageSensor = hardwareMap.voltageSensor.iterator().next()
    private var _voltage = voltageSensor.voltage
    val voltage by ::_voltage

    fun update() {
        _voltage = voltageSensor.voltage
    }

    override fun invoke(): Double {
        return voltage
    }
}