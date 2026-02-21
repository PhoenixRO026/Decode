package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class BlindDriveBlue : BlindDriveTurret() {
    override val pipeline = 1
}

@TeleOp
class BlindDriveRed : BlindDriveTurret() {
    override val pipeline = 2
}