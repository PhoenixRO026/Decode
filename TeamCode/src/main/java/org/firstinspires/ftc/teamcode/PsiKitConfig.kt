package org.firstinspires.ftc.teamcode

import android.content.Context
import com.qualcomm.ftccommon.FtcEventLoop
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop
import org.psilynx.psikit.ftc.FtcLogTuning
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLogSettings
import org.psilynx.psikit.ftc.wrappers.MotorWrapper

@Suppress("unused")
object PsiKitConfig {
    @JvmStatic
    @OnCreateEventLoop
    fun configure(context: Context, ftcEventLoop: FtcEventLoop) {
        PsiKitAutoLogSettings.apply {
            enabledByDefault = false
            enableLinearByDefault = false
        }
        FtcLogTuning.apply {
            prefetchBulkDataEachLoop = true
            nonBulkReadPeriodSec = 0.0
            logImu = false
            processColorDistanceSensorsInBackground = false
            logMotorCurrent = false
            motorCurrentReadPeriodSec = 0.1
            pinpointReadPeriodSec = 0.0
            pinpointLoggerCallsUpdate = false
            pinpointUseMinimalBulkReadScope = false
            pinpointWrapperPublishesOdometry = true
            pedroFollowerPublishesNamedOdometry = false
            prefetchOnlyControlHub = true
        }
    }
}