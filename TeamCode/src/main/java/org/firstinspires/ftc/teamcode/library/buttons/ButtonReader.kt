package org.firstinspires.ftc.teamcode.library.buttons

import java.util.function.BooleanSupplier


/**
 * Class that reads the value of button states.
 */
open class ButtonReader(buttonValue: BooleanSupplier) : KeyReader {
    /**
     * Last state of the button
     */
    private var lastState: Boolean

    /**
     * Current state of the button
     */
    private var currState: Boolean

    /**
     * the state of the button
     */
    private var buttonState: BooleanSupplier = buttonValue

    init {
        currState = buttonState.asBoolean
        lastState = currState
    }

    /**
     * Reads button value
     */
    override fun readValue() {
        lastState = currState
        currState = buttonState.asBoolean
    }

    override val isDown: Boolean
        /**
         * Checks if the button is down
         */
        get() = buttonState.asBoolean

    /**
     * Checks if the button was just pressed
     */
    override fun wasJustPressed(): Boolean {
        return (!lastState && currState)
    }

    /**
     * Checks if the button was just released
     */
    override fun wasJustReleased(): Boolean {
        return (lastState && !currState)
    }

    /**
     * Checks if the button state has changed
     */
    override fun stateJustChanged(): Boolean {
        return (lastState != currState)
    }
}