package org.firstinspires.ftc.teamcode.psikit

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.psilynx.psikit.core.Logger
import org.psilynx.psikit.ftc.FtcLoggingSession
import org.psilynx.psikit.ftc.autolog.PsiKitNoAutoLog

@PsiKitNoAutoLog
abstract class PsiKitOpMode : LinearOpMode() {
    /** Port for the optional RLOG server. Set to 0 to disable. */
    protected open val rlogPort: Int = 5800

    /** Output folder for RLOGWriter. */
    protected open val rlogFolder: String = "/sdcard/FIRST/PsiKit/"

    /** Optional filename override; blank means "use default". */
    protected open val rlogFilename: String = ""

    protected val psiKitSession: FtcLoggingSession = FtcLoggingSession()

    private var sessionStarted: Boolean = false

    override fun runOpMode() {
        ensurePsiKitStarted()
        runPsiKitOpMode()
        psiKitSession.end()
        sessionStarted = false
    }

    protected fun whileOpModeInInit(run: () -> Unit) {
        do {
            val beforeUserStart = Logger.getRealTimestamp()

            Logger.periodicBeforeUser()
            psiKitSession.logOncePerLoop(this)

            val beforeUserEnd = Logger.getRealTimestamp()
            if (opModeInInit())
                run()

            val afterUserStart = Logger.getRealTimestamp()
            Logger.periodicAfterUser(
                afterUserStart - beforeUserEnd,
                beforeUserEnd - beforeUserStart
            )
        } while (opModeInInit())
        ensurePsiKitStarted()
    }

    protected fun whileOpModeIsActive(run: () -> Unit) {
        do {
            val beforeUserStart = Logger.getRealTimestamp()

            Logger.periodicBeforeUser()
            psiKitSession.logOncePerLoop(this)

            val beforeUserEnd = Logger.getRealTimestamp()
            if (opModeIsActive())
                run()

            val afterUserStart = Logger.getRealTimestamp()
            Logger.periodicAfterUser(
                afterUserStart - beforeUserEnd,
                beforeUserEnd - beforeUserStart
            )
        } while (opModeIsActive())
    }

    private fun ensurePsiKitStarted() {
        if (sessionStarted) return

        if (rlogFilename.isNotBlank()) {
            psiKitSession.start(
                this,
                rlogPort,
                filename = rlogFilename,
                folder = rlogFolder,
                configure = { onPsiKitConfigureLogging() }
            )
        } else {
            psiKitSession.start(
                this,
                rlogPort,
                folder = rlogFolder,
                configure = { onPsiKitConfigureLogging() }
            )
        }

        sessionStarted = true
    }

    abstract fun runPsiKitOpMode()
    protected open fun onPsiKitConfigureLogging() {}
}