package org.firstinspires.ftc.teamcode.library.buttons

import java.util.function.BooleanSupplier

/**
 * Class gets the current state of a toggle button
 */
class ToggleButtonReader(
    private val buttonValue: BooleanSupplier,          // keep supplier so we can check physical state
    private var currToggleState: Boolean = false
) : ButtonReader(buttonValue) {

    // when true, the next wasJustReleased() will be consumed and won't flip currToggleState
    private var ignoreNextRelease: Boolean = false

    val state: Boolean
        get() {
            if (wasJustReleased()) {
                if (ignoreNextRelease) {
                    ignoreNextRelease = false
                } else {
                    currToggleState = !currToggleState
                }
            }
            return currToggleState
        }

    /**
     * Force the toggle to a specific state.
     * If consumeNextRelease == null (default), we consume the next release only if the physical
     * button is currently pressed. If you explicitly pass true/false you control consumption.
     */
    fun setState(value: Boolean, consumeNextRelease: Boolean? = null) {
        currToggleState = value
        val shouldConsume = consumeNextRelease ?: buttonValue.getAsBoolean()
        ignoreNextRelease = shouldConsume
    }

    /** Flip the toggle programmatically. */
    fun toggle(consumeNextRelease: Boolean? = null) {
        setState(!currToggleState, consumeNextRelease)
    }
}