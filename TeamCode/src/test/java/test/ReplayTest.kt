package test

import org.firstinspires.ftc.teamcode.teleop.TestPsiKit
import org.junit.Test
import org.junit.runner.RunWith
import org.psilynx.psikit.core.rlog.RLOGReplay
import org.psilynx.psikit.ftc.Replay
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config

@Config(shadows = [ShadowAppUtil::class])
@RunWith(RobolectricTestRunner::class)
class ReplayTest {
    @Test
    fun run() {
        val replaySource = RLOGReplay("log.rlog")
        Replay(TestPsiKit(), replaySource).run()
    }
}