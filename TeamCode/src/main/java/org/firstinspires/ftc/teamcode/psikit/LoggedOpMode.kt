package org.firstinspires.ftc.teamcode.psikit

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.psilynx.psikit.core.Logger
import org.psilynx.psikit.ftc.FtcLoggingSession
import org.psilynx.psikit.ftc.autolog.PsiKitNoAutoLog

@PsiKitNoAutoLog
abstract class LoggedOpMode : LinearOpMode() {
    /** Port for the optional RLOG server. Set to 0 to disable. */
    protected open val rlogPort: Int = 5800

    /** Output folder for RLOGWriter. */
    protected open val rlogFolder: String = "/sdcard/FIRST/PsiKit/"

    /** Optional filename override; blank means "use default". */
    protected open val rlogFilename: String = ""

    protected val psiKitSession: FtcLoggingSession = FtcLoggingSession()

    private var sessionStarted: Boolean = false

    private var beforeUserStart = 0.0
    private var beforeUserEnd = 0.0

    override fun runOpMode() {
        ensurePsiKitStarted()
        beforeUser()
        try {
            runLoggedOpMode()
        } finally {
            afterUser()
            psiKitSession.end()
            sessionStarted = false
        }
    }

    override fun waitForStart() {
        afterUser()
        if (!Logger.isReplay())
            super.waitForStart()
        beforeUser()
    }

    protected fun inInit(): Boolean {
        afterUser()
        beforeUser()
        return opModeInInit()
    }

    protected fun isActive(): Boolean {
        afterUser()
        beforeUser()
        return opModeIsActive()
    }

    private fun beforeUser() {
        beforeUserStart = Logger.getRealTimestamp()

        Logger.periodicBeforeUser()
        psiKitSession.logOncePerLoop(this)

        beforeUserEnd = Logger.getRealTimestamp()
    }

    private fun afterUser() {
        val afterUserStart = Logger.getRealTimestamp()
        Logger.periodicAfterUser(
            afterUserStart - beforeUserEnd,
            beforeUserEnd - beforeUserStart
        )
    }

    private fun ensurePsiKitStarted() {
        if (sessionStarted) return

        if (rlogFilename.isNotBlank()) {
            psiKitSession.start(
                this,
                rlogPort,
                filename = rlogFilename,
                folder = rlogFolder,
                configure = ::onPsiKitConfigureLogging
            )
        } else {
            psiKitSession.start(
                this,
                rlogPort,
                folder = rlogFolder,
                configure = ::onPsiKitConfigureLogging
            )
        }

        sessionStarted = true
    }

    abstract fun runLoggedOpMode()
    protected open fun onPsiKitConfigureLogging() {}
}