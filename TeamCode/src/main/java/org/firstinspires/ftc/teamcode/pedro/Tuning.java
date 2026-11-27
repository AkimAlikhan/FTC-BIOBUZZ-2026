package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * AutoTune procedures. Each @Tuner method shows up on the AutoTune web page
 * that the robot hosts. Run them top to bottom.
 */
public class Tuning {
    @Tuner(name = "1. Mecanum Tuner")
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner(name = "2. Pinpoint Tuner")
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner(name = "3. Foresight Tuner")
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                h -> new PinpointLocalizer(h, Constants.localizerConfig),
                h -> new Mecanum(h, Constants.drivetrainConfig));
    }

    // Localization, odometry, pose and driving tests work before Foresight is tuned.
    // Hold, line and curve tests need Foresight.
    @Tuner(name = "4. Tests")
    public static Procedure tests() {
        return new Tests(
                h -> new Mecanum(h, Constants.drivetrainConfig),
                h -> new PinpointLocalizer(h, Constants.localizerConfig),
                Constants.foresightTuned() ? () -> new Foresight(Constants.foresightConfig) : null);
    }
}
