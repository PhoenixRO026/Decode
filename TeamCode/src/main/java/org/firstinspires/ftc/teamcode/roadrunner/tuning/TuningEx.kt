@file:JvmName("TuningEx")
package org.firstinspires.ftc.teamcode.roadrunner.tuning

import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.EncoderGroup
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer

private const val goBILDA_4_BAR_POD: Float = 19.89436789f

fun fakeEncoderGroup(localizer: PinpointLocalizer) = object : EncoderGroup {
    private val driver = localizer.driver

    override val encoders: List<Encoder> = listOf(
        fakeMotorEncoder(
            { (driver.encoderX / goBILDA_4_BAR_POD).toInt() },
            { driver.getVelX(DistanceUnit.MM) }
        ),
        fakeMotorEncoder(
            { (driver.encoderY / goBILDA_4_BAR_POD).toInt() },
            { driver.getVelY(DistanceUnit.MM) }
        )
    )
    override val unwrappedEncoders: List<Encoder>
        get() = encoders

    override fun bulkRead() {
        driver.update()
    }

}

fun fakeMotorEncoder(position: () -> Int, velocity: () -> Double) = RawEncoder(
    object : DcMotorEx {
        override fun getManufacturer(): HardwareDevice.Manufacturer {
            TODO("Not yet implemented")
        }

        override fun getDeviceName(): String {
            TODO("Not yet implemented")
        }

        override fun getConnectionInfo(): String {
            TODO("Not yet implemented")
        }

        override fun getVersion(): Int {
            TODO("Not yet implemented")
        }

        override fun resetDeviceConfigurationForOpMode() {
            TODO("Not yet implemented")
        }

        override fun close() {
            TODO("Not yet implemented")
        }

        override fun setDirection(direction: DcMotorSimple.Direction?) {
            TODO("Not yet implemented")
        }

        override fun getDirection(): DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD

        override fun setPower(power: Double) {
            TODO("Not yet implemented")
        }

        override fun getPower(): Double {
            TODO("Not yet implemented")
        }

        override fun getMotorType(): MotorConfigurationType {
            TODO("Not yet implemented")
        }

        override fun setMotorType(motorType: MotorConfigurationType?) {
            TODO("Not yet implemented")
        }

        override fun getController(): DcMotorController {
            TODO("Not yet implemented")
        }

        override fun getPortNumber(): Int {
            TODO("Not yet implemented")
        }

        override fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior?) {
            TODO("Not yet implemented")
        }

        override fun getZeroPowerBehavior(): DcMotor.ZeroPowerBehavior {
            TODO("Not yet implemented")
        }

        override fun setPowerFloat() {
            TODO("Not yet implemented")
        }

        override fun getPowerFloat(): Boolean {
            TODO("Not yet implemented")
        }

        override fun setTargetPosition(position: Int) {
            TODO("Not yet implemented")
        }

        override fun getTargetPosition(): Int {
            TODO("Not yet implemented")
        }

        override fun isBusy(): Boolean {
            TODO("Not yet implemented")
        }

        override fun getCurrentPosition(): Int {
            return position()
        }

        override fun setMode(mode: DcMotor.RunMode?) {
            TODO("Not yet implemented")
        }

        override fun getMode(): DcMotor.RunMode {
            TODO("Not yet implemented")
        }

        override fun setMotorEnable() {
            TODO("Not yet implemented")
        }

        override fun setMotorDisable() {
            TODO("Not yet implemented")
        }

        override fun isMotorEnabled(): Boolean {
            TODO("Not yet implemented")
        }

        override fun setVelocity(angularRate: Double) {
            TODO("Not yet implemented")
        }

        override fun setVelocity(angularRate: Double, unit: AngleUnit?) {
            TODO("Not yet implemented")
        }

        override fun getVelocity(): Double {
            return velocity()
        }

        override fun getVelocity(unit: AngleUnit?): Double {
            TODO("Not yet implemented")
        }

        override fun setPIDCoefficients(mode: DcMotor.RunMode?, pidCoefficients: PIDCoefficients?) {
            TODO("Not yet implemented")
        }

        override fun setPIDFCoefficients(
            mode: DcMotor.RunMode?,
            pidfCoefficients: PIDFCoefficients?
        ) {
            TODO("Not yet implemented")
        }

        override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) {
            TODO("Not yet implemented")
        }

        override fun setPositionPIDFCoefficients(p: Double) {
            TODO("Not yet implemented")
        }

        override fun getPIDCoefficients(mode: DcMotor.RunMode?): PIDCoefficients {
            TODO("Not yet implemented")
        }

        override fun getPIDFCoefficients(mode: DcMotor.RunMode?): PIDFCoefficients {
            TODO("Not yet implemented")
        }

        override fun setTargetPositionTolerance(tolerance: Int) {
            TODO("Not yet implemented")
        }

        override fun getTargetPositionTolerance(): Int {
            TODO("Not yet implemented")
        }

        override fun getCurrent(unit: CurrentUnit?): Double {
            TODO("Not yet implemented")
        }

        override fun getCurrentAlert(unit: CurrentUnit?): Double {
            TODO("Not yet implemented")
        }

        override fun setCurrentAlert(current: Double, unit: CurrentUnit?) {
            TODO("Not yet implemented")
        }

        override fun isOverCurrent(): Boolean {
            TODO("Not yet implemented")
        }

    }
)