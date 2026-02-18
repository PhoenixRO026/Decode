package org.firstinspires.ftc.teamcode.psikit

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.MANUAL
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.psilynx.psikit.core.Logger
import org.psilynx.psikit.ftc.HardwareMapWrapper
import org.psilynx.psikit.ftc.OpModeControls
import org.psilynx.psikit.ftc.PsiKitLinearOpMode
import org.psilynx.psikit.ftc.wrappers.GamepadWrapper
import kotlin.time.measureTime

abstract class LoggedOpMode : PsiKitLinearOpMode(){
    override fun runOpMode() {
        preInit()
        _init()
        Logger.periodicAfterUser(0.0, 0.0)
        while (inInit) {
            Logger.periodicBeforeUser()
            updateHardware()
            _init_loop()
            Logger.periodicAfterUser(0.0, 0.0)
        }
        _start()
        while (isActive) {
            _loop()
        }
        _stop()
    }

    abstract fun _init()
    abstract fun _init_loop()
    abstract fun _start()
    abstract fun _loop()
    abstract fun _stop()

    private fun preInit() {
        Logger.reset()

        this.telemetry = MockTelemetry(this.telemetry)

        this.hardwareMap = HardwareMapWrapper(hardwareMap)

        allHubs = this.hardwareMap.getAll(LynxModule::class.java)

        allHubs.forEach {
            it.bulkCachingMode = MANUAL
        }
        this.gamepad1 = GamepadWrapper(this.gamepad1)
        this.gamepad2 = GamepadWrapper(this.gamepad2)
        val annotation = this::class.java.annotations.firstOrNull {
            it is Autonomous || it is TeleOp
        } ?: TeleOp::class
        Logger.recordMetadata(
            "OpMode Name",
            when(annotation){
                is Autonomous -> annotation.name.takeIf { it.isNotEmpty() } ?: this::class.java.simpleName
                is TeleOp     -> annotation.name.takeIf { it.isNotEmpty() } ?: this::class.java.simpleName
                else          -> error("Impossible")
            }
        )
        Logger.recordMetadata(
            "OpMode type",
            if(annotation is Autonomous) "Autonomous" else "TeleOp"
        )

        Logger.start()
    }

    private fun updateHardware() {
        processHardwareInputs()
    }

    private val inInit get() = !psiKitIsStarted && !psiKitIsStopRequested
    private val isActive get() = psiKitIsStarted && !psiKitIsStopRequested
}