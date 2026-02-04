package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.teleop.BoringDrive.BoringDriveConfig
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

    fun shootBall(rpm : Double) = SequentialAction(
        shooter.goToRpmAction(rpm),
        SleepAction(0.1.s),
        InstantAction{transfer.fingerUp()},
        SleepAction(0.2.s),
        InstantAction{transfer.fingerDown()},
        SleepAction(0.2.s),
    )

    fun intakeBalls(futureOuttakePos : Spindexer.TransferPos) = SequentialAction(
        InstantAction{intake.power = 0.75},
        transfer.goToPosAction(Spindexer.TransferPos.intake1),
        RaceAction(
            transfer.waitForColors(3.s),
            SleepAction(3.s)
        ),
        SleepAction(0.6.s),
        transfer.goToNextIntakeAction(),
        RaceAction(
            transfer.waitForColors(3.s),
            SleepAction(3.s)
        ),
        SleepAction(0.3.s),
        transfer.goToNextIntakeAction(),
        RaceAction(
            transfer.waitForColors(3.s),
            SleepAction(3.s)
        ),
        SleepAction(0.3.s),
        InstantAction{intake.power = 0.0},
        transfer.goToPosAction(futureOuttakePos),
        InstantAction{intake.power = -1.0}
    )

    fun shootBalls(rpm : Double, startPos : Spindexer.TransferPos) = SequentialAction(
        shooter.goToRpmAction(rpm),
        transfer.goToPosAction(startPos),
        shootBall(rpm),
        transfer.goToNextShootAction(),
        shootBall(rpm),
        transfer.goToNextShootAction(),
        shootBall(rpm),
        SleepAction(0.2.s),
        transfer.goToPosAction(Spindexer.TransferPos.intake1),
        shooter.goToRpmAction(0.0),
    )

    fun intakeTeleBalls() = SequentialAction(
        InstantAction{intake.power = 0.75},
        transfer.goToPosAction(Spindexer.TransferPos.intake1),
        RaceAction(
            transfer.waitForColors(3.s),
            SleepAction(3.s)
        ),
        SleepAction(0.6.s),
        transfer.goToNextIntakeAction(),
        RaceAction(
            transfer.waitForColors(3.s),
            SleepAction(3.s)
        ),
        SleepAction(0.3.s),
        transfer.goToNextIntakeAction(),
        RaceAction(
            transfer.waitForColors(3.s),
            SleepAction(3.s)
        ),
        SleepAction(0.3.s),
        InstantAction{intake.power = 0.0},
        transfer.goToPosAction(Spindexer.TransferPos.shoot1),
        InstantAction{intake.power = -1.0}
    )

    fun shootTeleBalls(rpm : Double, startPos : Spindexer.TransferPos) = SequentialAction(
        shooter.goToRpmAction(rpm),
        transfer.goToPosAction(startPos),
        shootBall(rpm),
        transfer.goToNextShootAction(),
        shootBall(rpm),
        transfer.goToNextShootAction(),
        shootBall(rpm),
        SleepAction(0.2.s),
        transfer.goToPosAction(Spindexer.TransferPos.intake1),
        shooter.goToRpmAction(0.0),
    )

    fun stopShootAction() = ParallelAction(
        InstantAction { shooter.goToRpmAction(0.0)}
    )

    fun stopIntakeAction() = ParallelAction(
        InstantAction{ intake.power = 0.0 }
    )

    init {
        val mecanumDrive = MecanumDrive(hardwareMap, pose.pose2d)

        ///  Shooter  ///

        // motors //
        val motorShooterTop = hardwareMap.get(DcMotorEx::class.java, "motorShooterTop")
        val motorShooterBottom = hardwareMap.get(DcMotorEx::class.java, "motorShooterBottom")
        val motorTurret = hardwareMap.get(DcMotorEx::class.java, "motorTurret")


        motorShooterTop.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorShooterTop.direction = DcMotorSimple.Direction.FORWARD
        motorShooterTop.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        motorShooterBottom.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorShooterBottom.direction = DcMotorSimple.Direction.REVERSE
        motorShooterBottom.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        motorTurret.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorTurret.direction = DcMotorSimple.Direction.FORWARD
        motorTurret.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // encoders //
        val encoderOuttake : Encoder = RawEncoder(mecanumDrive.rightBack)
        val encoderTurret : Encoder = RawEncoder(motorTurret)

        encoderOuttake.direction =DcMotorSimple.Direction.REVERSE

        // servos //
        val servoBackwall = hardwareMap.get(Servo::class.java, "servoBackwall")

        ///  Intake  ///

        // motors //
        val motorIntake = hardwareMap.get(DcMotorEx::class.java, "motorIntake")

        motorIntake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorIntake.direction = DcMotorSimple.Direction.FORWARD
        motorIntake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        ///  Transfer  ///

        // motors //
        val servoTransfer1 = hardwareMap.get(Servo::class.java, "servoTransfer1")
        val servoTransfer2 = hardwareMap.get(Servo::class.java, "servoTransfer2")
        val finger = hardwareMap.get(Servo::class.java, "finger")

        val colorSensor = hardwareMap.get(NormalizedColorSensor::class.java, "colorSensor")
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
            servoBackwall = servoBackwall,
            voltageSensor = voltageSensor
        )
        transfer = Spindexer(
            servoTransfer1 = servoTransfer1,
            servoTransfer2 = servoTransfer2,
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