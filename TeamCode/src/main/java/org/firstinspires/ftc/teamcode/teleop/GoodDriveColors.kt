package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class GoodDriveBlue : GoodDrive() {
    override val pipeline = 2
}

@TeleOp
class GoodDriveRed : GoodDrive() {
    override val pipeline = 1
}