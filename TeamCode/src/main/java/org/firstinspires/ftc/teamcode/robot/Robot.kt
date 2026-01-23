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
    val camera: CameraCore
    val limelight: LimeLightCore


    fun shootBall(rpm : Double) = SequentialAction(
        shooter.goToRpmAction(rpm),
        SleepAction(0.3.s),
        InstantAction{transfer.fingerUp()},
        SleepAction(0.2.s),
        InstantAction{transfer.fingerDown()},
        SleepAction(0.2.s),
    )

    fun intakeBalls(multiplier : Int, futureOuttakePos : Int) = SequentialAction(
        InstantAction{intake.power = 0.75},
        transfer.goToPosAction(BoringDriveConfig.pos, multiplier, BoringDriveConfig.intakeOffset),
        RaceAction(
            camera.waitForColors(3.s),
            SleepAction(3.s)
        ),
        SleepAction(0.5.s),
        transfer.goToPosAction(BoringDriveConfig.pos, multiplier + 1, 0.0),
        RaceAction(
            camera.waitForColors(3.s),
            SleepAction(3.s)
        ),
        SleepAction(0.3.s),
        transfer.goToPosAction(BoringDriveConfig.pos, multiplier + 2, 0.0),
        RaceAction(
            camera.waitForColors(3.s),
            SleepAction(3.s)
        ),
        SleepAction(0.4.s),
        InstantAction{intake.power = 0.0},
        transfer.goToPosAction(BoringDriveConfig.pos, futureOuttakePos, BoringDriveConfig.shooterOffset),
        InstantAction{intake.power = -1.0}
    )

    fun shootBalls(rpm : Double, multiplier : Int) = SequentialAction(
        shooter.goToRpmAction(rpm),
        transfer.goToPosAction(BoringDriveConfig.pos, multiplier, BoringDriveConfig.shooterOffset),
        shootBall(rpm),
        transfer.goToPosAction(BoringDriveConfig.pos, multiplier + 1, BoringDriveConfig.shooterOffset),
        shootBall(rpm),
        transfer.goToPosAction(BoringDriveConfig.pos, multiplier + 2, BoringDriveConfig.shooterOffset),
        shootBall(rpm),
        SleepAction(0.2.s),
        transfer.goToPosAction(BoringDriveConfig.pos, 0, 0.0),
        shooter.goToRpmAction(0.0),
    )

    fun intakeTeleBalls(multiplier : Int) = SequentialAction(
        InstantAction{intake.power = 0.75},
        transfer.goToPosAction(BoringDriveConfig.pos, multiplier, BoringDriveConfig.intakeOffset),
        RaceAction(
            camera.waitForColors(2.s),
            SleepAction(3.s)
        ),
        transfer.goToPosAction(BoringDriveConfig.pos, multiplier + 1, 0.0),
        RaceAction(
            camera.waitForColors(2.s),
            SleepAction(3.s)
        ),
        transfer.goToPosAction(BoringDriveConfig.pos, multiplier + 2, 0.0),
        RaceAction(
            camera.waitForColors(2.s),
            SleepAction(3.s)
        ),
        InstantAction{intake.power = 0.0},
        transfer.goToPosAction(BoringDriveConfig.pos, 0, BoringDriveConfig.shooterOffset),
    )

    fun shootTeleBalls(rpm : Double, multiplier : Int) = SequentialAction(
        shooter.goToRpmAction(rpm),
        SleepAction(0.9.s),
        transfer.goToPosAction(BoringDriveConfig.pos, multiplier, BoringDriveConfig.shooterOffset),
        shootBall(rpm),
        transfer.goToPosAction(BoringDriveConfig.pos, multiplier + 1, BoringDriveConfig.shooterOffset),
        shootBall(rpm),
        transfer.goToPosAction(BoringDriveConfig.pos, multiplier + 2, BoringDriveConfig.shooterOffset),
        shootBall(rpm),
        SleepAction(0.5 .s),
        transfer.goToPosAction(BoringDriveConfig.pos, 0, 0.0),
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

        val motorShooterTop = hardwareMap.get(DcMotorEx::class.java, "motorShooterTop")
        val motorShooterBottom = hardwareMap.get(DcMotorEx::class.java, "motorShooterBottom")

        motorShooterTop.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorShooterTop.direction = DcMotorSimple.Direction.FORWARD
        motorShooterTop.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        motorShooterBottom.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorShooterBottom.direction = DcMotorSimple.Direction.REVERSE
        motorShooterBottom.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE


        val encoderOuttake : Encoder = RawEncoder(mecanumDrive.rightBack)
        //val encoderOuttake : Encoder = OverflowEncoder(RawEncoder(mecanumDrive.rightBack))

        encoderOuttake.direction =DcMotorSimple.Direction.REVERSE

        val motorIntake = hardwareMap.get(DcMotorEx::class.java, "motorIntake")

        motorIntake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorIntake.direction = DcMotorSimple.Direction.FORWARD
        motorIntake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val motorTransfer = hardwareMap.get(DcMotorEx::class.java, "motorTransfer")

        motorTransfer.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorTransfer.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorTransfer.direction = DcMotorSimple.Direction.REVERSE
        motorTransfer.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val encoderTransfer : Encoder = RawEncoder(motorTransfer)
        encoderTransfer.direction = DcMotorSimple.Direction.FORWARD

        val finger = hardwareMap.get(Servo::class.java, "finger")

        val webcamColor = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        val limlit = hardwareMap.get(Limelight3A::class.java, "limelight")
        limlit.setPollRateHz(100)
        limlit.start()
        val voltageSensor = hardwareMap.voltageSensor.iterator().next()

        drive = Drive(mecanumDrive)
        shooter = Shooter(
            motorTop = motorShooterTop,
            motorBottom = motorShooterBottom,
            encoder = encoderOuttake,
            voltageSensor = voltageSensor
        )
        transfer = Spindexer(
            motor = motorTransfer,
            encoder = encoderTransfer,
            finger = finger
        )
        intake = Intake(
            motor = motorIntake
        )
        camera = CameraCore(
            cameraColor = webcamColor
        )
        limelight = LimeLightCore(
            camera = limlit,
            drive = drive
        )


    }
}