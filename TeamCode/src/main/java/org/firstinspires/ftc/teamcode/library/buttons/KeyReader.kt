package org.firstinspires.ftc.teamcode.library.buttons

interface KeyReader {
    /**
     * Reads button value
     */
    fun readValue()

    /**
     * Checks if the button is down
     */
    val isDown: Boolean

    /**
     * Checks if the button was just pressed
     */
    fun wasJustPressed(): Boolean

    /**
     * Checks if the button was just released
     */
    fun wasJustReleased(): Boolean

    /**
     * Checks if the button state has changed
     */
    fun stateJustChanged(): Boolean
}