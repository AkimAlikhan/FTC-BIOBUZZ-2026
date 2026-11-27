package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Constants {

    // ── Translational PIDF (fixes x/y error while on path) ───────────────────
    // Raise P if robot lags behind path, lower if it oscillates side-to-side
    public static double TRANS_P = 0.048;
    public static double TRANS_I = 0.0;
    public static double TRANS_D = 0.001;
    public static double TRANS_F = 0.027;

    // ── Heading PIDF (fixes rotation error while on path) ────────────────────
    // Raise P if robot heading is slow to correct, lower if it wiggles
    public static double HEAD_P = 0.56;
    public static double HEAD_I = 0.0;
    public static double HEAD_D = 0.001;
    public static double HEAD_F = 0.025;

    // ── Drive PIDF (controls speed along path) ────────────────────────────────
    // F and S are always required (they power the motors).
    // Once predictive braking is tuned, set DRIVE_P = 0 and DRIVE_D = 0.
    public static double DRIVE_P = 0.01;
    public static double DRIVE_I = 0.0;
    public static double DRIVE_D = 0;
    public static double DRIVE_F = 0.6;
    public static double DRIVE_S = 0.03;

    // ── Predictive Braking ────────────────────────────────────────────────────
    // STEP 1 — Run PredictiveBrakingTuner to fill PRED_K_LINEAR + PRED_K_QUAD.
    // STEP 2 — Tune PRED_KP via LineTest: raise until the end-hold gets tight
    //          but before jitter appears (good range: 0.05 – 0.3).
    // STEP 3 — After both are tuned: set DRIVE_P = 0, DRIVE_D = 0, CENTRIPETAL_SCALING = 0.
    public static double PRED_KP       = 0.075;    // holding strength at path end
    public static double PRED_K_LINEAR = 0.14523420300726744;   // ← replace with PredictiveBrakingTuner result
    public static double PRED_K_QUAD   = 1.0103761620598362E-4; // ← replace with PredictiveBrakingTuner result

    // ── Robot physics ─────────────────────────────────────────────────────────
    // Re-run ForwardZeroPowerAccelerationTuner / LateralZeroPowerAccelerationTuner
    // if you change wheels or robot weight
    public static double ROBOT_MASS                  = 15;
    public static double FORWARD_ZERO_POWER_ACCEL    = -39.28740365958303;
    public static double LATERAL_ZERO_POWER_ACCEL    = -59.129150249466996;
    // Set to 0 once predictive braking is fully tuned (centripetal + pred brake conflict)
    public static double CENTRIPETAL_SCALING         =   0;

    // ── Drive velocities (run MaxPowerTuner to get accurate values) ───────────
    public static double MAX_POWER   = 0.9;
    public static double X_VELOCITY  = 85.4461256612943;
    public static double Y_VELOCITY  = 56.81246347502461;

    // ── Pinpoint pod offsets (mm from robot center — measure carefully!) ──────
    // Wrong values here are the #1 cause of drifting paths
    // forwardPodY  : + = pod is to the LEFT  of robot center
    // strafePodX   : + = pod is in FRONT of robot center
    public static double FORWARD_POD_Y   =   2.9958375945804647;    // measured: mm from robot center, + = pod is LEFT of center
    public static double STRAFE_POD_X    = -5.682297113373529;   // measured: mm from robot center, - = pod is BEHIND center

    // ── Path constraints ──────────────────────────────────────────────────────
    // Lower MAX_VEL (e.g. 0.6–0.8) for tighter path accuracy on sharp curves
    public static double PATH_MAX_VEL        = 0.99;
    public static double PATH_MAX_ACCEL      = 100;
    public static double PATH_MAX_ANG_VEL    =   1;
    public static double PATH_MAX_ANG_ACCEL  =   1.0;

    // ─────────────────────────────────────────────────────────────────────────

    public static Follower createFollower(HardwareMap hardwareMap) {
        FollowerConstants fc = new FollowerConstants()
                .mass(ROBOT_MASS)
                .forwardZeroPowerAcceleration(FORWARD_ZERO_POWER_ACCEL)
                .lateralZeroPowerAcceleration(LATERAL_ZERO_POWER_ACCEL)
                .translationalPIDFCoefficients(
                        new PIDFCoefficients(TRANS_P, TRANS_I, TRANS_D, TRANS_F))
                .headingPIDFCoefficients(
                        new PIDFCoefficients(HEAD_P, HEAD_I, HEAD_D, HEAD_F))
                .drivePIDFCoefficients(
                        new FilteredPIDFCoefficients(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_F, DRIVE_S))
                .predictiveBrakingCoefficients(
                        new PredictiveBrakingCoefficients(PRED_KP, PRED_K_LINEAR, PRED_K_QUAD))
                .centripetalScaling(CENTRIPETAL_SCALING);

        MecanumConstants mc = new MecanumConstants()
                .maxPower(MAX_POWER)
                .rightFrontMotorName("fr")
                .rightRearMotorName("br")
                .leftRearMotorName("bl")
                .leftFrontMotorName("fl")
                .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                .xVelocity(X_VELOCITY)
                .yVelocity(Y_VELOCITY);

        PinpointConstants pc = new PinpointConstants()
                .forwardPodY(FORWARD_POD_Y)
                .strafePodX(STRAFE_POD_X)
                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName("pinpoint")
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

        return new FollowerBuilder(fc, hardwareMap)
                .pathConstraints(
                        new PathConstraints(PATH_MAX_VEL, PATH_MAX_ACCEL,
                                            PATH_MAX_ANG_VEL, PATH_MAX_ANG_ACCEL))
                .mecanumDrivetrain(mc)
                .pinpointLocalizer(pc)
                .build();
    }
}
