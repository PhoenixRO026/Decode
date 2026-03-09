package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
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
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.roadrunner.FusedLocalizer
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive


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
    val fusedLocalizer: FusedLocalizer

    fun init(deg: Double){
        transfer.goToPos(Spindexer.TransferPos.intake0)
        shooter.goToPos(shooter.degToTick(deg))
    }

    fun intakeBalls(nextShoot: Spindexer.TransferPos) = SequentialAction (
        intake.startIntakeAction(),
        transfer.waitForColors(2.0.s),
        transfer.goToNextIntakeAction(),
        SleepAction(0.1.s),
        transfer.waitForColors(1.0.s),
        transfer.goToNextIntakeAction(),
        SleepAction(0.1.s),
        transfer.waitForColors(1.0.s),
        transfer.goToPosAction(nextShoot),
        SleepAction(0.15.s),
        intake.spew(),
    )

    fun intakeTeleBallsSort() = SequentialAction (
        transfer.goToPosAction(Spindexer.TransferPos.intake0),
        intake.startIntakeAction(),
        transfer.waitForColors(7.0.s),
        InstantAction{transfer.updateBallSlot()},
        transfer.goToNextIntakeAction(),
        SleepAction(0.6.s),
        transfer.waitForColors(7.0.s),
        InstantAction{transfer.updateBallSlot()},
        transfer.goToNextIntakeAction(),
        SleepAction(0.6.s),
        transfer.waitForColors(7.0.s),
        InstantAction{transfer.updateBallSlot()},
        transfer.goToPosAction(Spindexer.TransferPos.shoot0),
        intake.spew(),
        SleepAction(0.6.s),
    )
    fun intakeTeleBallsRaw() = SequentialAction (
        transfer.goToPosAction(Spindexer.TransferPos.intake0),
        intake.startIntakeAction(),
        transfer.waitForDistance(7.0.s),
        SleepAction(0.5.s),
        transfer.goToNextIntakeAction(),
        transfer.waitForDistance(7.0.s),
        SleepAction(0.5.s),
        transfer.goToNextIntakeAction(),
        transfer.waitForDistance(7.0.s),
        SleepAction(0.5.s),
        intake.spew(),
        transfer.goToPosAction(Spindexer.TransferPos.shoot0),
        SleepAction(0.2.s),
    )

    fun shootBall() = SequentialAction(
        SleepAction(0.2.s),
        InstantAction{transfer.fingerUp()},
        SleepAction(0.2.s),
        InstantAction{transfer.fingerDown()},
        SleepAction(0.2.s),
        InstantAction{transfer.emptySlot(transfer.currentPos)}
    )

    fun shootBalls(rpm: Double = shooter.rpmFar) = SequentialAction (
        SleepAction(0.55.s),
        shooter.goToRpmAction(rpm),
        shootBall(),
        transfer.goToNextShootAction(),
        SleepAction(0.1.s),
        shootBall(),
        transfer.goToNextShootAction(),
        SleepAction(0.1.s),
        shootBall(),
        ParallelAction(
            transfer.goToPosAction(Spindexer.TransferPos.intake0),
            shooter.goToRpmAction(shooter.rpmRest)
        )
    )

    fun shootBallsRaw(rpm: Double = shooter.rpmFar) = SequentialAction (
        SleepAction(0.55.s),
        shooter.goToRpmAction(rpm),
        shootBall(),
        transfer.goToNextShootAction(),
        SleepAction(0.1.s),
        shootBall(),
        transfer.goToNextShootAction(),
        SleepAction(0.1.s),
        shootBall(),
        ParallelAction(
            transfer.goToPosAction(Spindexer.TransferPos.intake0),
            shooter.goToRpmAction(shooter.rpmRest)
        )
    )

    fun shootBallsTele() = SequentialAction (
        transfer.goToPosAction(Spindexer.TransferPos.shoot0),
        shooter.goToRpmAction(shooter.targetRpm),
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
        val limlit = hardwareMap.get(Limelight3A::class.java, "limelight")
        limlit.setPollRateHz(60)
        limlit.start()

        fusedLocalizer = FusedLocalizer(
            hardwareMap,
            pose.pose2d,
            limlit,
            { shooter.turretAngle.asRad }
        )

        val mecanumDrive = MecanumDrive(hardwareMap, pose.pose2d, fusedLocalizer.pinpointLocalizer)

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
        val encoderTurret : Encoder = RawEncoder(mecanumDrive.rightBack)

        encoderOuttake.direction = DcMotorSimple.Direction.FORWARD
        encoderTurret.direction = DcMotorSimple.Direction.REVERSE


        ///  Intake  ///

        // motors //
        val motorIntake = hardwareMap.get(DcMotorEx::class.java, "motorIntake")

        motorIntake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorIntake.direction = DcMotorSimple.Direction.FORWARD
        motorIntake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        ///  Transfer  ///

        // servos  //
        val servoTransferFront = hardwareMap.get(Servo::class.java, "servoTransferFront")
        val servoTransferBack = hardwareMap.get(Servo::class.java, "servoTransferBack")
        val finger = hardwareMap.get(Servo::class.java, "finger")

        val colorSensor = hardwareMap.get(NormalizedColorSensor::class.java, "colorSensor")
        colorSensor.gain = 15f

        val voltageSensor = hardwareMap.get(VoltageSensor::class.java, "Control Hub")

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