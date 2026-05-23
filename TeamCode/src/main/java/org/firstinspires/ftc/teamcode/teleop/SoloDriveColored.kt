package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class SoloDriveRed : HappyDrive() {
    override val pipeline = 2
}

@TeleOp
class SoloDriveBlue : HappyDrive() {
    override val pipeline = 1
}