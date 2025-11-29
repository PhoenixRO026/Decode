package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.commonlibs.units.Pose
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import kotlin.jvm.java

class Robot(
    hardwareMap: HardwareMap,
    pose: Pose = Pose(0.0.cm, 0.0.cm, 0.0.deg),
    resetEncoders: Boolean = true
) {
    val drive: Drive
    val shooter: Shooter
    val transfer: Spindexer
    val intake: Intake

    init {
        val mecanumDrive = MecanumDrive(hardwareMap, pose.pose2d)

        val motorShooterTop = hardwareMap.get(DcMotorEx::class.java, "motorShooterTop")
        val motorShooterBottom = hardwareMap.get(DcMotorEx::class.java, "motorShooterBottom")

        motorShooterTop.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorShooterTop.direction = DcMotorSimple.Direction.FORWARD
        motorShooterTop.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        motorShooterBottom.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorShooterBottom.direction = DcMotorSimple.Direction.REVERSE
        motorShooterBottom.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val encoderOuttake : Encoder = RawEncoder(mecanumDrive.rightBack)

        val motorIntake = hardwareMap.get(DcMotorEx::class.java, "motorIntake")

        motorIntake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorIntake.direction = DcMotorSimple.Direction.FORWARD
        motorIntake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val motorTransfer = hardwareMap.get(DcMotorEx::class.java, "motorTransfer")

        motorTransfer.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorTransfer.direction = DcMotorSimple.Direction.FORWARD
        motorTransfer.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val encoderTransfer : Encoder = RawEncoder(mecanumDrive.leftFront)

        val finger = hardwareMap.get(Servo::class.java, "finger")


        drive = Drive(mecanumDrive)
        shooter = Shooter(
            motorTop = motorShooterTop,
            motorBottom = motorShooterBottom,
            encoder = encoderOuttake
        )
        transfer = Spindexer(
            motor = motorTransfer,
            encoder = encoderTransfer,
            finger = finger
        )
        intake = Intake(
            motor = motorIntake
        )

    }
}