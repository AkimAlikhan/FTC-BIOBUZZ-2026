package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 3.0 robot constants (2026-27 BIOBUZZ).
 *
 * A Follower is built from three parts:
 *   Localizer  -> where the robot is   (PinpointLocalizer + PinpointConfig)
 *   Drivetrain -> how it moves         (Mecanum + MecanumConfig)
 *   Algorithm  -> how it follows paths (Foresight + ForesightConfig)
 *
 * Every value marked TODO comes from AutoTune (robot Wi-Fi -> AutoTune page),
 * in this order: Mecanum Tuner -> Pinpoint Tuner -> Foresight Tuner -> Tests.
 * Each tuner prints a Java block. Paste it over the matching config below.
 */
public class Constants {

    // Same drivetrain as 2025-26: names and directions carried over.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.backLeftName.set("bl");
        c.frontRightName.set("fr");
        c.backRightName.set("br");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

        c.manualBrakeMode.set(true);
    });

    // Same odometry as 2025-26. Values carried over from the old Pedro 2.x PinpointConstants:
    //   forwardPodY  2.9958 -> xPodOffset   (X/forward pod: inches LEFT of center)
    //   strafePodX  -5.6823 -> yPodOffset   (Y/strafe pod: inches FORWARD of center, negative = behind)
    //   forward encoder FORWARD, strafe encoder REVERSED, goBILDA 4-bar pods, inches.
    // Check them with AutoTune "4. Tests" -> Localization / Odometry before tuning Foresight.
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.xPodOffset.set(2.9958375945804647);
        c.yPodOffset.set(-5.682297113373529);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    });

    /** Localizer only. Works before Foresight is tuned. */
    public static PinpointLocalizer createLocalizer(HardwareMap h) {
        return new PinpointLocalizer(h, localizerConfig);
    }

    /** Drivetrain only. Works before Foresight is tuned. */
    public static Mecanum createDrivetrain(HardwareMap h) {
        return new Mecanum(h, drivetrainConfig);
    }

    /*
     * TODO Foresight Tuner: replace null with the block it prints, which looks like:
     *
     *   public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
     *       c.forwardTranslational.set(...);  c.strafeTranslational.set(...);
     *       c.coast.set(...);                 c.brake.set(...);
     *       c.headingFeedback.set(...);       c.headingBrakeCoefficients.set(...);
     *       c.linearBrakeCoefficients.set(...); c.quadraticBrakeCoefficients.set(...);
     *       c.maxAchievableForwardVelocity.set(...); c.maxAchievableStrafeVelocity.set(...);
     *       c.naturalForwardDeceleration.set(...);   c.naturalStrafeDeceleration.set(...);
     *   });
     *
     * Those fields are required. ForesightConfig rejects a config without them,
     * so this stays null until it is tuned, rather than holding guessed values.
     */
    public static ForesightConfig foresightConfig = null;

    public static boolean foresightTuned() {
        return foresightConfig != null;
    }

    public static Follower create(HardwareMap h) {
        if (!foresightTuned()) {
            throw new IllegalStateException(
                    "Foresight is not tuned yet. Run the Foresight Tuner in AutoTune and paste its config into Constants.foresightConfig.");
        }
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
