package org.firstinspires.ftc.teamcode.library.buttons

import java.util.function.BooleanSupplier


/**
 * Class gets the current state of a toggle button
 */
class ToggleButtonReader(buttonValue: BooleanSupplier) : ButtonReader(buttonValue) {
    private var currToggleState: Boolean

    init {
        currToggleState = false
    }

    val state: Boolean
        get() {
            if (wasJustReleased()) {
                currToggleState = !currToggleState
            }
            return (currToggleState)
        }
}