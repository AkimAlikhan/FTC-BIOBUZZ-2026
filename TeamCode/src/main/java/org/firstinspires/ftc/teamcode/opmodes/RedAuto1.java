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
 * RED AUTO 1
 *
 *   1  shoot preload             10 shoot
 *   2  path 1                    11 intake off
 *   3  intake on                 12 path 5
 *   4  path 2 at half speed      13 intake on, wait 1 s
 *   5  shoot                     14 intake off
 *   6  path 3 (full speed)       15 path 6
 *   7  intake on                 16 shoot
 *   8  path 4                    17 path 7
 *   9  wait 1 s (intake on)      18 stop everything
 *
 * The final pose + alliance go to the blackboard for TeleOp.
 * Needs a tuned Foresight config (Constants.create throws until then).
 */
@Autonomous(name = "RED AUTO 1", group = "Competition", preselectTeleOp = "TeleOp")
public class RedAuto1 extends OpMode {

    private static final boolean IS_RED = true;

    public static double SLOW_SPEED = 0.5;   // path 2 (fraction of full speed)
    public static double WAIT_SEC   = 1.0;   // steps 9 and 13

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
        return line(new Pose(8.367, 9.058), new Pose(46.093, 116.541)).reverseTangent();
    }

    private Path path4() {
        return line(new Pose(46.093, 116.541), new Pose(40.316, 131.622))
                .linear(Math.toRadians(-110), Math.toRadians(70));
    }

    private Path path5() {
        return line(new Pose(40.316, 131.622), new Pose(10.05, 54.598))
                .linear(Math.toRadians(70), Math.toRadians(-165));
    }

    private Path path6() {
        return line(new Pose(10.05, 54.598), new Pose(32.097, 109.342)).reverseTangent();
    }

    private Path path7() {
        return line(new Pose(32.097, 109.342), new Pose(15.635, 96.415)).tangent();
    }

    // ── Sequence ─────────────────────────────────────────────────────────────
    private enum Step {
        SHOOT_PRELOAD, WAIT_SHOT_1,
        PATH_1,
        PATH_2,
        WAIT_SHOT_2,
        PATH_3,
        PATH_4, WAIT_AFTER_4, WAIT_SHOT_3,
        PATH_5, WAIT_AFTER_5,
        PATH_6, WAIT_SHOT_4,
        PATH_7,
        DONE
    }

    private Follower follower;
    private Shooter shooter;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private Step step = Step.SHOOT_PRELOAD;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(START);
        follower.update();

        shooter = new Shooter(hardwareMap, true); // turret must face forward at init
        shooter.setAlliance(IS_RED);

        telemetry.addLine("RED AUTO 1");
        telemetry.addData("Start", "x %.1f  y %.1f  h %.0f°", START.x(), START.y(), Math.toDegrees(START.heading()));
        telemetry.addLine("Turret must face straight forward now (encoder zeroed).");
        telemetry.update();
    }

    @Override
    public void start() {
        matchTimer.reset();
        go(Step.SHOOT_PRELOAD);
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.pose();
        if (step != Step.DONE) shooter.update(pose, follower.velocity()); // after step 18 everything stays off

        boolean pathDone = !follower.isBusy();
        boolean shotDone = !shooter.isBusy();

        switch (step) {
            case SHOOT_PRELOAD:                       // 1
                shooter.shoot();
                go(Step.WAIT_SHOT_1);
                break;
            case WAIT_SHOT_1:
                if (shotDone) { follower.follow(path1()); go(Step.PATH_1); }   // 2
                break;
            case PATH_1:
                if (pathDone) {
                    shooter.setIntake(1);             // 3
                    follower.follow(path2());         // 4 (half speed)
                    go(Step.PATH_2);
                }
                break;
            case PATH_2:
                if (pathDone) {
                    shooter.setIntake(0);
                    shooter.shoot();                  // 5
                    go(Step.WAIT_SHOT_2);
                }
                break;
            case WAIT_SHOT_2:
                if (shotDone) { follower.follow(path3()); go(Step.PATH_3); }   // 6
                break;
            case PATH_3:
                if (pathDone) {
                    shooter.setIntake(1);             // 7
                    follower.follow(path4());         // 8
                    go(Step.PATH_4);
                }
                break;
            case PATH_4:
                if (pathDone) go(Step.WAIT_AFTER_4);  // 9 (intake still on)
                break;
            case WAIT_AFTER_4:
                if (timer.seconds() > WAIT_SEC) {
                    shooter.setIntake(0);
                    shooter.shoot();                  // 10
                    go(Step.WAIT_SHOT_3);
                }
                break;
            case WAIT_SHOT_3:
                if (shotDone) {
                    shooter.setIntake(0);             // 11
                    follower.follow(path5());         // 12
                    go(Step.PATH_5);
                }
                break;
            case PATH_5:
                if (pathDone) {
                    shooter.setIntake(1);             // 13
                    go(Step.WAIT_AFTER_5);
                }
                break;
            case WAIT_AFTER_5:
                if (timer.seconds() > WAIT_SEC) {
                    shooter.setIntake(0);             // 14
                    follower.follow(path6());         // 15
                    go(Step.PATH_6);
                }
                break;
            case PATH_6:
                if (pathDone) {
                    shooter.shoot();                  // 16
                    go(Step.WAIT_SHOT_4);
                }
                break;
            case WAIT_SHOT_4:
                if (shotDone) { follower.follow(path7()); go(Step.PATH_7); }   // 17
                break;
            case PATH_7:
                if (pathDone) {
                    follower.stop();                  // 18
                    shooter.stop();
                    go(Step.DONE);
                }
                break;
            case DONE:
                break;
        }

        // Keep the blackboard current, so TeleOp gets a pose even if Auto is cut off.
        Field.savePose(blackboard, pose, IS_RED);

        telemetry.addLine("RED AUTO 1");
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

    private void go(Step s) {
        step = s;
        timer.reset();
    }
}
