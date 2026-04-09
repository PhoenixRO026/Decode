package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class SadDriveBlue : SadDrive() {
    override val pipeline = 1
}

@TeleOp
class SadDriveRed : SadDrive() {
    override val pipeline = 2
}