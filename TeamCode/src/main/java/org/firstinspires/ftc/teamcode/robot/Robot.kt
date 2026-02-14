package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.commonlibs.units.Pose
import com.commonlibs.units.SleepAction
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.commonlibs.units.ms
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

    data object RobotConfig {
        var ticksPerRev = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0)
        var pos = ticksPerRev / 3.0
        var multiplier = 0
        var shooterOffset = 94.0
        var intakeOffset = 0.0
    }

    fun shootBall(rpm : Double) = SequentialAction(
        SleepAction(0.1.s),
        InstantAction{transfer.fingerUp()},
        SleepAction(0.2.s),
        InstantAction{transfer.fingerDown()},
        SleepAction(0.1.s),
    )

    fun intakeBalls(futureOuttakePos : Int) = SequentialAction(
        transfer.goToPosAction(RobotConfig.pos, 0, RobotConfig.intakeOffset),
        camera.waitForColors(5.0.s),
        SleepAction(0.08.s),
        transfer.goToPosAction(RobotConfig.pos, 1, 0.0),
        camera.waitForColors(3.0.s),

        SleepAction(0.08.s),
        transfer.goToPosAction(RobotConfig.pos, 2, 0.0),
        camera.waitForColors(2.0.s),

        SleepAction(0.2.s),
        InstantAction{intake.power = 0.0},
        transfer.goToPosAction(RobotConfig.pos, futureOuttakePos, RobotConfig.shooterOffset),
    )

    fun shootBalls(rpm : Double, multiplier : Int) = SequentialAction(
        transfer.goToPosAction(RobotConfig.pos, multiplier, RobotConfig.shooterOffset),
        shootBall(rpm),
        transfer.goToPosAction(RobotConfig.pos, multiplier + 1, RobotConfig.shooterOffset),
        shootBall(rpm),
        transfer.goToPosAction(RobotConfig.pos, multiplier + 2, RobotConfig.shooterOffset),
        shootBall(rpm),
        SleepAction(0.2.s),
        transfer.goToPosAction(RobotConfig.pos, 0, 0.0),
        shooter.goToRpmAction(0.0),
    )

    fun shootBallsSlow(rpm : Double, multiplier : Int) = SequentialAction(
        transfer.goToPosAction(RobotConfig.pos, multiplier, RobotConfig.shooterOffset),
        shootBall(rpm),
        SleepAction(0.1.s),
        transfer.goToPosAction(RobotConfig.pos, multiplier + 1, RobotConfig.shooterOffset),
        shootBall(rpm),
        SleepAction(0.1.s),
        transfer.goToPosAction(RobotConfig.pos, multiplier + 2, RobotConfig.shooterOffset),
        shootBall(rpm),
        SleepAction(0.2.s),
        transfer.goToPosAction(RobotConfig.pos, 0, 0.0),
        shooter.goToRpmAction(0.0),
    )

    fun intakeTeleBalls(multiplier : Int) = SequentialAction(
        InstantAction{intake.power = 1.0},
        transfer.goToPosAction(RobotConfig.pos, multiplier, RobotConfig.intakeOffset),
        camera.waitForColors(7.s),
        SleepAction(0.07.s),
        transfer.goToPosAction(RobotConfig.pos, multiplier + 1, 0.0),
        camera.waitForColors(7.s),
        SleepAction(0.07.s),
        transfer.goToPosAction(RobotConfig.pos, multiplier + 2, 0.0),
        camera.waitForColors(7.s),
        SleepAction(0.07.s),
        InstantAction{intake.power = 0.0},
    )

    fun shootTeleBalls(rpm : Double, multiplier : Int) = SequentialAction(
        //shooter.goToRpmAction(rpm),
        //SleepAction(0.9.s),
        transfer.goToPosAction(RobotConfig.pos, multiplier, RobotConfig.shooterOffset),
        shootBall(rpm),
        transfer.goToPosAction(RobotConfig.pos, multiplier + 1, RobotConfig.shooterOffset),
        shootBall(rpm),
        transfer.goToPosAction(RobotConfig.pos, multiplier + 2, RobotConfig.shooterOffset),
        shootBall(rpm),
        SleepAction(0.3 .s),
        transfer.goToPosAction(RobotConfig.pos, 0, 0.0),
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

        val limlit = hardwareMap.get(Limelight3A::class.java, "limelight")
        limlit.setPollRateHz(100)
        limlit.start()
        val webcamColor = hardwareMap.get(WebcamName::class.java, "Webcam 1")
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
            finger = finger,
        )
        intake = Intake(
            motor = motorIntake
        )
        limelight = LimeLightCore(
            camera = limlit,
            drive = drive
        )
        camera = CameraCore(
            cameraColor = webcamColor
        )



    }
}