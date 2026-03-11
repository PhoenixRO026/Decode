package org.firstinspires.ftc.teamcode.library.pedro.localization;

import static com.commonlibs.units.DistanceKt.copy;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.library.pedro.math.Matrix;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;

import java.util.NavigableMap;
import java.util.TreeMap;

public class FusionLocalizer implements Localizer {
    private final PinpointLocalizer deadReckoning;
    private Pose2d currentPosition;
    private PoseVelocity2d currentVelocity;
    private Matrix P; //State Covariance
    private final Matrix Q; //Process Noise Covariance
    private final Matrix R; //Measurement Noise Covariance
    private long lastUpdateTime = -1;
    private final NavigableMap<Long, Pose2d> poseHistory = new TreeMap<>();
    private final NavigableMap<Long, PoseVelocity2d> twistHistory = new TreeMap<>();
    private final NavigableMap<Long, Matrix> covarianceHistory = new TreeMap<>();
    private final int bufferSize;
    private double previousVisionTimestamp = 0.0;

//    public FusionLocalizer(
//            PinpointLocalizer deadReckoning,
//            Covariance initialCovariance,
//            Covariance processVariance,
//            Covariance measurementVariance,
//            int bufferSize,
//            Pose2d initialPose
//    ) {
//        this(deadReckoning, initialCovariance, processVariance, measurementVariance, bufferSize);
//        setStartPose(initialPose);
//    }

    public FusionLocalizer(
            PinpointLocalizer deadReckoning,
            Covariance initialCovariance,
            Covariance processVariance,
            Covariance measurementVariance,
            int bufferSize
    ) {
        this.deadReckoning = deadReckoning;
        this.currentPosition = new Pose2d(0.0, 0.0, 0.0);

        //Standard Deviations for Kalman Filter
        this.P = Matrix.diag(initialCovariance.getX(), initialCovariance.getY(), initialCovariance.getHeading());
        this.Q = Matrix.diag(processVariance.getX(), processVariance.getY(), processVariance.getHeading());
        this.R = Matrix.diag(measurementVariance.getX(), measurementVariance.getY(), measurementVariance.getHeading());
        this.bufferSize = bufferSize;
        twistHistory.put(0L, new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
    }

    @Override
    public PoseVelocity2d update() {
        //Updates odometry
        PoseVelocity2d robotVel = deadReckoning.update();
        long now = System.nanoTime();
        double dt = lastUpdateTime < 0 ? 0 : (now - lastUpdateTime) / 1e9;
        lastUpdateTime = now;

        //Updates twist, note that the dead reckoning localizer returns world-frame twist
        PoseVelocity2d twist = deadReckoning.getWorldVelocity();
        twistHistory.put(now, copy(twist));
        currentVelocity = copy(twist);

        //Perform twist integration to propagate the fused position estimate based on how the odometry thinks the robot has moved
        currentPosition = integrate(currentPosition, twist, dt);

        //Update Kalman Filter
        updateCovariance(dt);

        poseHistory.put(now, copy(currentPosition));
        covarianceHistory.put(now, P.copy());
        if (poseHistory.size() > bufferSize) poseHistory.pollFirstEntry();
        if (twistHistory.size() > bufferSize) twistHistory.pollFirstEntry();
        if (covarianceHistory.size() > bufferSize) covarianceHistory.pollFirstEntry();

        return robotVel;
    }

    /**
     * Consider the system xₖ₊₁ = xₖ + (f(xₖ, uₖ) + wₖ) * Δt.
     * <p>
     * wₖ is the noise in the system caused by sensor uncertainty, a zero-mean random vector with covariance Q.
     * <p>
     * The Kalman Filter update step is given by:
     * <pre>
     *     Pₖ₊₁ = F * Pₖ * Fᵀ + G * Q * Gᵀ
     * </pre>
     * Here F and G represent the State Transition Matrix and Control-to-State Matrix respectively.
     * <p>
     * The State Transition Matrix F is given by I + ∂f/∂x.
     * We computed our twist integration using a first-order forward-Euler approximation.
     * Therefore, f only depends on the twist, not on x, so ∂f/∂x = 0 and F = I.
     * <p>
     * The Control-to-State Matrix G is given by ∂xₖ₊₁ / ∂wₖ.
     * Here this is simply I * Δt.
     * <p>
     * The Kalman update is Pₖ₊₁ = F * Pₖ * Fᵀ + G * Q * Gᵀ.
     * With F = I and G = I * Δt, we get Pₖ₊₁ = Q * Δt².
     *
     * @param dt the time step Δt in seconds
     */
    private void updateCovariance(double dt) {
        Matrix G = Matrix.createRotation(getPose().heading.log()).multiply(dt);
        P = P.plus(G.multiply(Q.multiply(G.transposed())));
    }

    /**
     * Adds a vision measurement using the default measurement variance
     * @param measuredPose the measured position by the camera, enter NaN to a specific axis if the camera couldn't measure that axis
     * @param timestamp the timestamp of the measurement
     */
    public void addMeasurement(Pose2d measuredPose, long timestamp, double llts) {
        addMeasurement(measuredPose, timestamp, llts, null);
    }

    /**
     * Adds a vision measurement with a custom variance for this specific measurement
     * @param measuredPose the measured position by the camera, enter NaN to a specific axis if the camera couldn't measure that axis
     * @param timestamp the timestamp of the measurement
     * @param measurementVariance the variance for this specific measurement (x, y, heading), or null to use the default
     */
    public void addMeasurement(Pose2d measuredPose, long timestamp, double llts, Covariance measurementVariance) {
        if (llts == previousVisionTimestamp) {
            return;
        }
        previousVisionTimestamp = llts;

        Matrix measurementR = measurementVariance == null
                ? R
                : Matrix.diag(measurementVariance.getX(), measurementVariance.getY(), measurementVariance.getHeading());
        // Reject if timestamp is outside our poseHistory time window
        if (poseHistory.isEmpty() || timestamp < poseHistory.firstKey() || timestamp > poseHistory.lastKey())
            return;

        Pose2d pastPose = interpolate(timestamp, poseHistory);
        if (pastPose == null)
            pastPose = currentPosition;

        // Measurement residual y = z - x
        boolean measX = !Double.isNaN(measuredPose.position.x);
        boolean measY = !Double.isNaN(measuredPose.position.y);
        boolean measH = !Double.isNaN(measuredPose.heading.log());

        Matrix y = new Matrix(new double[][]{
                {measX ? measuredPose.position.x - pastPose.position.x : 0},
                {measY ? measuredPose.position.y - pastPose.position.y : 0},
                {measH ? normalizeAngleSigned(measuredPose.heading.log() - pastPose.heading.log()) : 0}
        });

        // Measurement mask M
        Matrix M = Matrix.diag(
                measX ? 1 : 0,
                measY ? 1 : 0,
                measH ? 1 : 0
        );

        // Covariance at measurement time
        Matrix Pm = covarianceHistory.floorEntry(timestamp).getValue();

        // Innovation covariance S = P + R
        Matrix S = Pm.plus(measurementR);

        // Apply gain K = P * (P + R)^(-1)
        Matrix K = Pm.multiply(S.inverse());

        // Apply mask
        K = M.multiply(K);
        y = M.multiply(y);

        // State update
        Matrix Ky = K.multiply(y);
        Pose2d updatedPast = new Pose2d(
                pastPose.position.x + Ky.get(0, 0),
                pastPose.position.y + Ky.get(1, 0),
                pastPose.heading.log() + Ky.get(2, 0)
        );
        poseHistory.put(timestamp, updatedPast);

        // Joseph-form covariance update
        Matrix I = Matrix.identity(3);
        Matrix IK = I.minus(K);
        Matrix updatedCovariance =
                IK.multiply(Pm).multiply(IK.transposed())
                        .plus(K.multiply(measurementR).multiply(K.transposed()));

        covarianceHistory.put(timestamp, updatedCovariance);

        // Forward propagate pose + covariance
        long prevTime = timestamp;
        Pose2d prevPose = updatedPast;
        Matrix prevCov = updatedCovariance;

        for (NavigableMap.Entry<Long, Pose2d> entry :
                poseHistory.tailMap(timestamp, false).entrySet()) {

            long t = entry.getKey();
            PoseVelocity2d twist = interpolateVel(t, twistHistory);
            if (twist == null)
                twist = currentVelocity;

            double dt = (t - prevTime) / 1e9;

            Pose2d nextPose = integrate(prevPose, twist, dt);
            poseHistory.put(t, nextPose);

            // Covariance propagation: P ← P + Q dt²
            Matrix G = Matrix.createRotation(prevPose.heading.log()).multiply(dt);
            prevCov = prevCov.plus(G.multiply(Q.multiply(G.transposed())));
            covarianceHistory.put(t, prevCov);

            prevPose = nextPose;
            prevTime = t;
        }

        currentPosition = copy(poseHistory.lastEntry().getValue());
        P = covarianceHistory.lastEntry().getValue().copy();
    }


    //Performs linear interpolation inside the history map for the value at a given timestamp
    private static PoseVelocity2d interpolateVel(long timestamp, NavigableMap<Long, PoseVelocity2d> history) {
        Long lowerKey = history.floorKey(timestamp);
        Long upperKey = history.ceilingKey(timestamp);

        if (lowerKey == null || upperKey == null) return null;
        if (lowerKey.equals(upperKey)) return copy(history.get(lowerKey));

        PoseVelocity2d lowerPose = history.get(lowerKey);
        PoseVelocity2d upperPose = history.get(upperKey);

        double ratio = (double) (timestamp - lowerKey) / (upperKey - lowerKey);

        double x = lowerPose.linearVel.x + ratio * (upperPose.linearVel.x - lowerPose.linearVel.x);
        double y = lowerPose.linearVel.y + ratio * (upperPose.linearVel.y - lowerPose.linearVel.y);
        double headingDiff = getSmallestAngleDifference(upperPose.angVel, lowerPose.angVel);
        double heading = lowerPose.angVel + ratio * headingDiff;

        return new PoseVelocity2d(new Vector2d(x, y), heading);
    }
    //Performs linear interpolation inside the history map for the value at a given timestamp
    private static Pose2d interpolate(long timestamp, NavigableMap<Long, Pose2d> history) {
        Long lowerKey = history.floorKey(timestamp);
        Long upperKey = history.ceilingKey(timestamp);

        if (lowerKey == null || upperKey == null) return null;
        if (lowerKey.equals(upperKey)) return copy(history.get(lowerKey));

        Pose2d lowerPose = history.get(lowerKey);
        Pose2d upperPose = history.get(upperKey);

        double ratio = (double) (timestamp - lowerKey) / (upperKey - lowerKey);

        double x = lowerPose.position.x + ratio * (upperPose.position.x - lowerPose.position.x);
        double y = lowerPose.position.y + ratio * (upperPose.position.y - lowerPose.position.y);
        double headingDiff = getSmallestAngleDifference(upperPose.heading.log(), lowerPose.heading.log());
        double heading = lowerPose.heading.log() + ratio * headingDiff;

        return new Pose2d(x, y, heading);
    }

    private Pose2d integrate(Pose2d previousPose, PoseVelocity2d twist, double dt) {
        //Standard forward-Euler first-order approximation for twist integration
        double dx = twist.linearVel.x * dt;
        double dy = twist.linearVel.y * dt;
        double dTheta = twist.angVel * dt;

        return new Pose2d(
                previousPose.position.x + dx,
                previousPose.position.y + dy,
                previousPose.heading.log() + dTheta
        );
    }

    @Override
    public Pose2d getPose() {
        return currentPosition;
    }

    public void setStartPose(Pose2d setStart) {
        deadReckoning.setPose(setStart);
        poseHistory.put(0L, copy(setStart));
        covarianceHistory.put(0L, P.copy());
        currentPosition = copy(setStart);
    }

    @Override
    public void setPose(Pose2d setPose) {
        currentPosition = copy(setPose);
        deadReckoning.setPose(setPose);

        if (poseHistory.lastEntry() != null)
            poseHistory.lastEntry().setValue(copy(setPose));
        else
            setStartPose(setPose);
    }

    public static double normalizeAngleSigned(double angleRadians) {
        double angle = normalizeAngle(angleRadians);
        if (angle >= Math.PI) {
            return angle - 2*Math.PI;
        }
        return angle;
    }

    public static double getSmallestAngleDifference(double one, double two) {
        return Math.min(normalizeAngle(one - two), normalizeAngle(two - one));
    }

    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (2*Math.PI);
        if (angle < 0) {
            return angle + 2*Math.PI;
        }
        return angle;
    }
}