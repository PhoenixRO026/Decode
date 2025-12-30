package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.acmerobotics.roadrunner.ftc.FlightRecorder.write
import com.acmerobotics.roadrunner.ftc.throwIfModulesAreOutdated
import com.acmerobotics.roadrunner.now
import com.acmerobotics.roadrunner.range
import com.commonlibs.units.Pose
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.roadrunner.Drawing
import org.firstinspires.ftc.teamcode.roadrunner.Localizer
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer
import org.firstinspires.ftc.teamcode.roadrunner.messages.DriveCommandMessage
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumCommandMessage
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage
import java.util.LinkedList
import kotlin.math.ceil
import kotlin.math.max

class MecanumDriveEx(
    hardwareMap: HardwareMap,
    pose: Pose,
    private val voltageProvider: () -> Double = object : () -> Double {
        val voltageSensor = hardwareMap.voltageSensor.iterator().next()
        override fun invoke(): Double {
            return voltageSensor.voltage
        }
    }
) {
    @Config
    data object MecanumDriveExParams {
        // drive model parameters
        @JvmField var inPerTick: Double = 1.0 / 25.4
        @JvmField var lateralInPerTick: Double = inPerTick
        @JvmField var trackWidthTicks: Double = 293.771977566021

        // feedforward parameters (in tick units)
        @JvmField var kS: Double = 1.3262172742865515
        @JvmField var kV: Double = 0.005331590914669022
        @JvmField var kA: Double = 0.00266

        // path profile parameters (in inches)
        @JvmField var maxWheelVel: Double = 50.0
        @JvmField var minProfileAccel: Double = -40.0
        @JvmField var maxProfileAccel: Double = 80.0

        // turn profile parameters (in radians)
        @JvmField var maxAngVel: Double = Math.PI // shared with path
        @JvmField var maxAngAccel: Double = Math.PI

        // path controller gains
        @JvmField var axialGain: Double = 15.0
        @JvmField var lateralGain: Double = 15.0
        @JvmField var headingGain: Double = 12.0 // shared with turn

        @JvmField var axialVelGain: Double = 0.0
        @JvmField var lateralVelGain: Double = 0.0
        @JvmField var headingVelGain: Double = 0.0 // shared with turn
    }

    val kinematics = MecanumKinematics(
        MecanumDriveExParams.inPerTick * MecanumDriveExParams.trackWidthTicks,
        MecanumDriveExParams.inPerTick / MecanumDriveExParams.lateralInPerTick
    )

    val defaultTurnConstraints = TurnConstraints(
        MecanumDriveExParams.maxAngVel,
        -MecanumDriveExParams.maxAngAccel,
        MecanumDriveExParams.maxAngAccel
    )
    val defaultVelConstraint = MinVelConstraint(listOf(
            kinematics.WheelVelConstraint(MecanumDriveExParams.maxWheelVel),
            AngularVelConstraint(MecanumDriveExParams.maxAngVel)
    ))
    val defaultAccelConstraint = ProfileAccelConstraint(
        MecanumDriveExParams.minProfileAccel,
        MecanumDriveExParams.maxProfileAccel
    )

    val leftFront: DcMotorEx
    val leftBack: DcMotorEx
    val rightBack: DcMotorEx
    val rightFront: DcMotorEx

    val localizer: Localizer
    private val poseHistory = LinkedList<Pose2d>()

    private val estimatedPoseWriter = DownsampledWriter("ESTIMATED_POSE", 50000000)
    private val targetPoseWriter = DownsampledWriter("TARGET_POSE", 50000000)
    private val driveCommandWriter = DownsampledWriter("DRIVE_COMMAND", 50000000)
    private val mecanumCommandWriter = DownsampledWriter("MECANUM_COMMAND", 50000000)

    init {
        throwIfModulesAreOutdated(hardwareMap)

        for (module in hardwareMap.getAll<LynxModule>(LynxModule::class.java)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
        }

        leftFront = hardwareMap.get(DcMotorEx::class.java, "motorLF")
        leftBack = hardwareMap.get(DcMotorEx::class.java, "motorLB")
        rightBack = hardwareMap.get(DcMotorEx::class.java, "motorRB")
        rightFront = hardwareMap.get(DcMotorEx::class.java, "motorRF")

        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        leftFront.direction = DcMotorSimple.Direction.REVERSE
        leftBack.direction = DcMotorSimple.Direction.REVERSE

        localizer = PinpointLocalizer(hardwareMap, MecanumDriveExParams.inPerTick, pose.pose2d)

        write("MECANUM_PARAMS", MecanumDriveExParams)
    }

    fun setDrivePowers(powers: PoseVelocity2d) {
        val wheelVels = MecanumKinematics(1.0).inverse<Time>(
            PoseVelocity2dDual.constant(powers, 1)
        )

        var maxPowerMag = 1.0
        for (power in wheelVels.all()) {
            maxPowerMag = max(maxPowerMag, power.value())
        }

        leftFront.power = wheelVels.leftFront[0] / maxPowerMag
        leftBack.power = wheelVels.leftBack[0] / maxPowerMag
        rightBack.power = wheelVels.rightBack[0] / maxPowerMag
        rightFront.power = wheelVels.rightFront[0] / maxPowerMag
    }

    inner class FollowTrajectoryAction @JvmOverloads constructor(
        val timeTrajectory: TimeTrajectory,
        correctionTimeSec: Double = 0.0
    ) : Action {
        private var beginTs = -1.0

        private val xPoints: DoubleArray
        private val yPoints: DoubleArray
        private val correctionTimeSec: Double

        init {
            val disps: List<Double> = range(
                0.0, timeTrajectory.path.length(),
                max(2, ceil(timeTrajectory.path.length() / 2).toInt())
            )
            xPoints = DoubleArray(disps.size)
            yPoints = DoubleArray(disps.size)
            for (i in disps.indices) {
                val p = timeTrajectory.path[disps[i], 1].value()
                xPoints[i] = p.position.x
                yPoints[i] = p.position.y
            }

            this.correctionTimeSec = correctionTimeSec
        }

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }

            if (t >= timeTrajectory.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0

                return false
            }

            val txWorldTarget: Pose2dDual<Time> = timeTrajectory[t]
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot: PoseVelocity2d = updatePoseEstimate()

            val command: PoseVelocity2dDual<Time> = HolonomicController(
                MecanumDriveExParams.axialGain,
                MecanumDriveExParams.lateralGain,
                MecanumDriveExParams.headingGain,
                MecanumDriveExParams.axialVelGain,
                MecanumDriveExParams.lateralVelGain,
                MecanumDriveExParams.headingVelGain
            )
                .compute(txWorldTarget, localizer.getPose(), robotVelRobot)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels: MecanumKinematics.WheelVelocities<Time> =
                kinematics.inverse(command)
            val voltage: Double = voltageProvider()

            val feedforward = MotorFeedforward(
                MecanumDriveExParams.kS,
                MecanumDriveExParams.kV / MecanumDriveExParams.inPerTick,
                MecanumDriveExParams.kA / MecanumDriveExParams.inPerTick
            )
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
            mecanumCommandWriter.write(
                MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
                )
            )

            leftFront.power = leftFrontPower
            leftBack.power = leftBackPower
            rightBack.power = rightBackPower
            rightFront.power = rightFrontPower

            p.put("x", localizer.getPose().position.x)
            p.put("y", localizer.getPose().position.y)
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()))

            val error = txWorldTarget.value().minusExp(localizer.getPose())
            p.put("xError", error.position.x)
            p.put("yError", error.position.y)
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()))

            // only draw when active; only one drive action should be active at a time
            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, localizer.getPose())

            c.setStroke("#4CAF50FF")
            c.setStrokeWidth(1)
            c.strokePolyline(xPoints, yPoints)

            return true
        }

        override fun preview(fieldOverlay: Canvas) {
            fieldOverlay.setStroke("#4CAF507A")
            fieldOverlay.setStrokeWidth(1)
            fieldOverlay.strokePolyline(xPoints, yPoints)
        }
    }

    inner class TurnAction(private val turn: TimeTurn) : Action {
        private var beginTs = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }

            if (t >= turn.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0

                return false
            }

            val txWorldTarget: Pose2dDual<Time> = turn[t]
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot: PoseVelocity2d = updatePoseEstimate()

            val command: PoseVelocity2dDual<Time> = HolonomicController(
                MecanumDriveExParams.axialGain,
                MecanumDriveExParams.lateralGain,
                MecanumDriveExParams.headingGain,
                MecanumDriveExParams.axialVelGain,
                MecanumDriveExParams.lateralVelGain,
                MecanumDriveExParams.headingVelGain
            )
                .compute(txWorldTarget, localizer.getPose(), robotVelRobot)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels: MecanumKinematics.WheelVelocities<Time> =
                kinematics.inverse(command)
            val voltage: Double = voltageProvider()
            val feedforward = MotorFeedforward(
                MecanumDriveExParams.kS,
                MecanumDriveExParams.kV / MecanumDriveExParams.inPerTick,
                MecanumDriveExParams.kA / MecanumDriveExParams.inPerTick
            )
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
            mecanumCommandWriter.write(
                MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
                )
            )

            leftFront.power = feedforward.compute(wheelVels.leftFront) / voltage
            leftBack.power = feedforward.compute(wheelVels.leftBack) / voltage
            rightBack.power = feedforward.compute(wheelVels.rightBack) / voltage
            rightFront.power = feedforward.compute(wheelVels.rightFront) / voltage

            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, localizer.getPose())

            c.setStroke("#7C4DFFFF")
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)

            return true
        }

        override fun preview(fieldOverlay: Canvas) {
            fieldOverlay.setStroke("#7C4DFF7A")
            fieldOverlay.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)
        }
    }

    fun updatePoseEstimate(): PoseVelocity2d {
        val vel = localizer.update()
        poseHistory.add(localizer.getPose())

        while (poseHistory.size > 100) {
            poseHistory.removeFirst()
        }

        estimatedPoseWriter.write(PoseMessage(localizer.getPose()))

        return vel
    }

    private fun drawPoseHistory(c: Canvas) {
        val xPoints = DoubleArray(poseHistory.size)
        val yPoints = DoubleArray(poseHistory.size)

        var i = 0
        for (t in poseHistory) {
            xPoints[i] = t.position.x
            yPoints[i] = t.position.y

            i++
        }

        c.setStrokeWidth(1)
        c.setStroke("#3F51B5")
        c.strokePolyline(xPoints, yPoints)
    }

    fun actionBuilder(beginPose: Pose2d, correctionTimeSec: Double = 0.0): TrajectoryActionBuilder {
        return TrajectoryActionBuilder(
            { turn: TimeTurn -> TurnAction(turn) },
            { timeTrajectory: TimeTrajectory ->
                FollowTrajectoryAction(
                    timeTrajectory,
                    correctionTimeSec
                )
            },
            TrajectoryBuilderParams(
                1e-6,
                ProfileParams(
                    0.25, 0.1, 1e-2
                )
            ),
            beginPose, 0.0,
            defaultTurnConstraints,
            defaultVelConstraint, defaultAccelConstraint
        )
    }
}