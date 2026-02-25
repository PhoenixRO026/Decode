package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class HappyDriveBlue : HappyDrive() {
    override val pipeline = 1
}

@TeleOp
class HappyDriveRed : HappyDrive() {
    override val pipeline = 2
}