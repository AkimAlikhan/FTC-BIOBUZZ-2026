package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Field;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * BLUE AUTO 2
 *
 *   1  shoot preload
 *   2  drive the path
 *   3  stop everything
 *
 * Same route as RED AUTO 2, rotated 180° around the field center
 * (x -> 144 - x, y -> 144 - y, heading -> heading + 180°). Edit freely.
 *
 * The final pose + alliance go to the blackboard for TeleOp.
 * Needs a tuned Foresight config (Constants.create throws until then).
 */
@Autonomous(name = "BLUE AUTO 2", group = "Competition", preselectTeleOp = "TeleOp")
public class BlueAuto2 extends OpMode {

    private static final boolean IS_RED = false;

    // ── Poses (inches, degrees) ──────────────────────────────────────────────
    private static final Pose START = new Pose(111.639, 10.719, Math.toRadians(90));

    private Path path() {
        return line(new Pose(111.639, 10.719), new Pose(132.391, 21.822))
                .constant(Math.toRadians(90));
    }

    // ── Sequence ─────────────────────────────────────────────────────────────
    private enum Step { SHOOT_PRELOAD, WAIT_SHOT, PATH, DONE }

    private Follower follower;
    private Shooter shooter;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private Step step = Step.SHOOT_PRELOAD;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(START);
        follower.update();

        shooter = new Shooter(hardwareMap, true); // turret must face forward at init
        shooter.setAlliance(IS_RED);

        telemetry.addLine("BLUE AUTO 2");
        telemetry.addData("Start", "x %.1f  y %.1f  h %.0f°", START.x(), START.y(), Math.toDegrees(START.heading()));
        telemetry.addLine("Turret must face straight forward now (encoder zeroed).");
        telemetry.update();
    }

    @Override
    public void start() {
        matchTimer.reset();
        step = Step.SHOOT_PRELOAD;
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.pose();
        if (step != Step.DONE) shooter.update(pose, follower.velocity()); // after step 3 everything stays off

        switch (step) {
            case SHOOT_PRELOAD:                                   // 1
                shooter.shoot();
                step = Step.WAIT_SHOT;
                break;
            case WAIT_SHOT:
                if (!shooter.isBusy()) {
                    follower.follow(path());                      // 2
                    step = Step.PATH;
                }
                break;
            case PATH:
                if (!follower.isBusy()) {
                    follower.stop();                              // 3
                    shooter.stop();
                    step = Step.DONE;
                }
                break;
            case DONE:
                break;
        }

        // Keep the blackboard current, so TeleOp gets a pose even if Auto is cut off.
        Field.savePose(blackboard, pose, IS_RED);

        telemetry.addLine("BLUE AUTO 2");
        telemetry.addData("Step", step);
        telemetry.addData("Time", "%.1f s", matchTimer.seconds());
        telemetry.addData("Robot", "x %.1f  y %.1f  h %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Follower", "%s  %.0f%%", follower.mode(), follower.completion() * 100);
        shooter.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) Field.savePose(blackboard, follower.pose(), IS_RED);
        if (shooter != null) shooter.stop();
    }
}
