package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.curve;
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
 * RED AUTO 3
 *
 *   1  shoot preload
 *   2  path 1
 *   3  intake on
 *   4  path 2 at half speed
 *   5  shoot
 *   6  path 3 (full speed)
 *   7  stop everything
 *
 * The final pose + alliance go to the blackboard for TeleOp.
 * Needs a tuned Foresight config (Constants.create throws until then).
 */
@Autonomous(name = "RED AUTO 3", group = "Competition", preselectTeleOp = "TeleOp")
public class RedAuto3 extends OpMode {

    private static final boolean IS_RED = true;

    public static double SLOW_SPEED = 0.5;   // path 2 (fraction of full speed)

    // ── Poses (inches, degrees) ──────────────────────────────────────────────
    private static final Pose START = new Pose(55.912, 8.287, Math.toRadians(90));

    private Path path1() {
        return curve(new Pose(55.912, 8.287), new Pose(36.677, 16.539), new Pose(16.462, 11.683))
                .linear(Math.toRadians(90), Math.toRadians(-100));
    }

    private Path path2() {
        return line(new Pose(16.462, 11.683), new Pose(8.367, 9.058))
                .constant(Math.toRadians(-100))
                .with(Constants.foresightConfig.maxPathSpeed.at(SLOW_SPEED));
    }

    private Path path3() {
        return curve(new Pose(8.367, 9.058), new Pose(28.356, 61.774), new Pose(13.083, 93.595))
                .reverseTangent();
    }

    // ── Sequence ─────────────────────────────────────────────────────────────
    private enum Step {
        SHOOT_PRELOAD, WAIT_SHOT_1,
        PATH_1,
        PATH_2,
        WAIT_SHOT_2,
        PATH_3,
        DONE
    }

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

        telemetry.addLine("RED AUTO 3");
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
        if (step != Step.DONE) shooter.update(pose); // after step 7 everything stays off

        boolean pathDone = !follower.isBusy();
        boolean shotDone = !shooter.isBusy();

        switch (step) {
            case SHOOT_PRELOAD:                       // 1
                shooter.shoot();
                step = Step.WAIT_SHOT_1;
                break;
            case WAIT_SHOT_1:
                if (shotDone) { follower.follow(path1()); step = Step.PATH_1; }   // 2
                break;
            case PATH_1:
                if (pathDone) {
                    shooter.setIntake(1);             // 3
                    follower.follow(path2());         // 4 (half speed)
                    step = Step.PATH_2;
                }
                break;
            case PATH_2:
                if (pathDone) {
                    shooter.setIntake(0);
                    shooter.shoot();                  // 5
                    step = Step.WAIT_SHOT_2;
                }
                break;
            case WAIT_SHOT_2:
                if (shotDone) { follower.follow(path3()); step = Step.PATH_3; }   // 6 (full speed)
                break;
            case PATH_3:
                if (pathDone) {
                    follower.stop();                  // 7
                    shooter.stop();
                    step = Step.DONE;
                }
                break;
            case DONE:
                break;
        }

        // Keep the blackboard current, so TeleOp gets a pose even if Auto is cut off.
        Field.savePose(blackboard, pose, IS_RED);

        telemetry.addLine("RED AUTO 3");
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
