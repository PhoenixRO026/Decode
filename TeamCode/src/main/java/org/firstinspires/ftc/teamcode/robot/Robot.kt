package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.commonlibs.units.Pose
import com.commonlibs.units.SleepAction
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.commonlibs.units.s
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
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
    val limelight: LimeLightCore

    fun intakeBalls(nextShoot: Spindexer.TransferPos) = SequentialAction (
        intake.startIntakeAction(),
        transfer.waitForColors(5.0.s),
        InstantAction{transfer.updateBallSlot()},
        transfer.goToNextIntakeAction(),
        SleepAction(0.65.s),
        transfer.waitForColors(5.0.s),
        InstantAction{transfer.updateBallSlot()},
        transfer.goToNextIntakeAction(),
        SleepAction(0.65.s),
        transfer.waitForColors(5.0.s),
        InstantAction{transfer.updateBallSlot()},
        transfer.goToPosAction(nextShoot),
        SleepAction(0.5.s),
        intake.spew(),
    )

    fun intakeTeleBalls() = SequentialAction (
        transfer.goToPosAction(Spindexer.TransferPos.intake0),
        intake.startIntakeAction(),
        transfer.waitForColors(7.0.s),
        InstantAction{transfer.updateBallSlot()},
        transfer.goToNextIntakeAction(),
        SleepAction(0.65.s),
        transfer.waitForColors(7.0.s),
        InstantAction{transfer.updateBallSlot()},
        transfer.goToNextIntakeAction(),
        SleepAction(0.65.s),
        transfer.waitForColors(7.0.s),
        InstantAction{transfer.updateBallSlot()},
        transfer.goToPosAction(Spindexer.TransferPos.shoot0),
        SleepAction(0.5.s),
    )

    fun shootBall() = SequentialAction(
        SleepAction(0.15.s),
        InstantAction{transfer.fingerUp()},
        SleepAction(0.4.s),
        InstantAction{transfer.fingerDown()},
        SleepAction(0.15.s),
        InstantAction{transfer.emptySlot(transfer.currentPos)}
    )

    fun shootBalls() = SequentialAction (
        shooter.goToRpmAction(shooter.rpmClose),
        shootBall(),
        transfer.goToNextShootAction(),
        shootBall(),
        transfer.goToNextShootAction(),
        shootBall(),
        transfer.goToPosAction(Spindexer.TransferPos.intake0)
    )

    fun shootPurple() = SequentialAction (
        transfer.goToPurpleAction(),
        shootBall()
    )

    fun shootGreen() = SequentialAction (
        transfer.goToGreenAction(),
        shootBall()
    )

    init {
        val mecanumDrive = MecanumDrive(hardwareMap, pose.pose2d)

        ///  Shooter  ///

        // motors //
        val motorShooterTop = hardwareMap.get(DcMotorEx::class.java, "motorShooterTop")
        val motorShooterBottom = hardwareMap.get(DcMotorEx::class.java, "motorShooterBottom")
        val motorTurret = hardwareMap.get(DcMotorEx::class.java, "motorTurret")

        motorShooterTop.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorShooterTop.direction = DcMotorSimple.Direction.REVERSE
        motorShooterTop.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        motorShooterBottom.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorShooterBottom.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorShooterBottom.direction = DcMotorSimple.Direction.FORWARD
        motorShooterBottom.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        motorTurret.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorTurret.direction = DcMotorSimple.Direction.REVERSE
        motorTurret.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // encoders //
        val encoderOuttake : Encoder = RawEncoder(motorShooterBottom)
        val encoderTurret : Encoder = RawEncoder(motorTurret)

        encoderOuttake.direction =DcMotorSimple.Direction.REVERSE
        encoderTurret.direction = DcMotorSimple.Direction.REVERSE


        ///  Intake  ///

        // motors //
        val motorIntake = hardwareMap.get(DcMotorEx::class.java, "motorIntake")

        motorIntake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorIntake.direction = DcMotorSimple.Direction.REVERSE
        motorIntake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        ///  Transfer  ///

        // servos  //
        val servoTransferFront = hardwareMap.get(Servo::class.java, "servoTransferFront")
        val servoTransferBack = hardwareMap.get(Servo::class.java, "servoTransferBack")
        val finger = hardwareMap.get(Servo::class.java, "finger")

        val colorSensor = hardwareMap.get(NormalizedColorSensor::class.java, "colorSensor")
        colorSensor.gain = 1.0f

        val limlit = hardwareMap.get(Limelight3A::class.java, "limelight")
        limlit.setPollRateHz(100)
        limlit.start()

        val voltageSensor = hardwareMap.voltageSensor.iterator().next()

        drive = Drive(mecanumDrive)
        shooter = Shooter(
            motorTop = motorShooterTop,
            motorBottom = motorShooterBottom,
            motorTurret = motorTurret,
            encoderOuttake = encoderOuttake,
            encoderTurret = encoderTurret,
            voltageSensor = voltageSensor
        )
        transfer = Spindexer(
            servoTransfer1 = servoTransferFront,
            servoTransfer2 = servoTransferBack,
            finger = finger,
            colorSensor = colorSensor
        )
        intake = Intake(
            motor = motorIntake
        )
        limelight = LimeLightCore(
            camera = limlit,
            drive = drive
        )
    }
}