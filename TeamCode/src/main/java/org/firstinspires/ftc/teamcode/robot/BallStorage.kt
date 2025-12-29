package org.firstinspires.ftc.teamcode.robot

import com.commonlibs.units.Angle
import kotlin.math.absoluteValue

class BallStorage(
    firstStorage: Storage = Storage.NONE,
    secondStorage: Storage = Storage.NONE,
    thirdStorage: Storage = Storage.NONE
) {
    enum class Storage {
        PURPLE,
        GREEN,
        NONE
    }

    private val array = arrayOf(firstStorage, secondStorage, thirdStorage)

    fun hasGreen() = array.contains(Storage.GREEN)
    fun hasPurple() = array.contains(Storage.PURPLE)

    fun getGreenIndexes() = array.filter { it == Storage.GREEN }.mapIndexed { index, _ -> index }
    fun getPurpleIndexes() = array.filter { it == Storage.GREEN }.mapIndexed { index, _ -> index }

    fun getGreenShooterPositions() = getGreenIndexes().map { getShooterPositionByIndex(it) }
    fun getPurpleShooterPositions() = getPurpleIndexes().map { getShooterPositionByIndex(it) }

    fun getClosestGreenShooterPosition(currentAngle: Angle): Spindexer.Position {
        return getGreenShooterPositions().minBy { it.angle.diffTo(currentAngle).asDeg.absoluteValue }
    }
    fun getClosestPurpleShooterPosition(currentAngle: Angle): Spindexer.Position {
        return getPurpleShooterPositions().minBy { it.angle.diffTo(currentAngle).asDeg.absoluteValue }
    }

    fun removeBall(index: Int) {
        array[index] = Storage.NONE
    }

    fun removeBall(position: Spindexer.Position) {
        removeBall(position.index)
    }

    fun getShooterPositionByIndex(index: Int): Spindexer.Position {
        return when (index) {
            0 -> Spindexer.Position.SHOOTER_1
            1 -> Spindexer.Position.SHOOTER_2
            else -> Spindexer.Position.SHOOTER_3
        }
    }
}