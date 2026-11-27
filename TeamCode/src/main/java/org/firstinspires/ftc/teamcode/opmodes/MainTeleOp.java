package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Field;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * BIOBUZZ competition TeleOp.
 *
 * Uses the Pedro 3.0 Pinpoint localizer + Mecanum drivetrain directly, so it runs
 * BEFORE Foresight is tuned (no Follower needed for driving and aiming).
 *
 * INIT:  DPAD LEFT = RED,  DPAD RIGHT = BLUE  (defaults to the alliance the last Auto ran)
 *
 * GAMEPAD 1
 *   Left stick      drive (field-centric, from this alliance's driver station)
 *   Right stick X   turn
 *   Left bumper     SHOOT: rev flywheel -> open gate -> intake 1 s -> close gate, stop all
 *   Right trigger   intake in              Left trigger   intake out
 *   Triangle / Y    reset pose to the alliance start pose
 *
 * Driving is never blocked, including while shooting.
 * Shooter/turret settings live in subsystems/Shooter.java.
 */
@TeleOp(name = "TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {

    // ── Start pose (RED; blue is mirrored through the field center). Triangle resets to this. ─
    public static double RED_START_X = 55.912, RED_START_Y = 8.287, RED_START_HEADING_DEG = 90;

    // ── Field-centric: the field direction each driver faces. Red drivers stand at the
    // left wall (look toward +x), blue drivers at the right wall (look toward -x).
    public static double RED_DRIVER_FORWARD_DEG  = 0;
    public static double BLUE_DRIVER_FORWARD_DEG = 180;

    protected Localizer localizer;
    protected Mecanum drivetrain;
    protected Shooter shooter;

    protected boolean isRed = true;
    private boolean lastTriangle = false;
    private Pose autoPose;

    // ── Hooks for the test / tuning TeleOps that extend this one ─────────────
    /** true = start from the pose (and alliance) the last Auto saved. */
    protected boolean useSavedPose() { return true; }
    /** true = save the pose at the end, for the next OpMode. */
    protected boolean savePoseOnStop() { return true; }
    /** Shown at the top of telemetry. */
    protected String title() { return "TeleOp"; }
    /** Called every loop before the shooter updates (extra controls). */
    protected void extraControls(Pose pose) {}
    /** Called every loop after the normal telemetry. */
    protected void extraTelemetry() {}

    @Override
    public void init() {
        localizer  = Constants.createLocalizer(hardwareMap);
        drivetrain = Constants.createDrivetrain(hardwareMap);

        // After an Auto: keep its pose, alliance and turret encoder count.
        // No Auto since the app started: the turret must face forward now (encoder is zeroed).
        autoPose = useSavedPose() ? Field.loadPose(blackboard) : null;
        isRed    = useSavedPose() ? Field.loadRed(blackboard, true) : true;
        shooter  = new Shooter(hardwareMap, autoPose == null);
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_left)  isRed = true;
        if (gamepad1.dpad_right) isRed = false;
        telemetry.addLine(title());
        telemetry.addData("ALLIANCE", isRed ? "RED  (dpad ◄)" : "BLUE (dpad ►)");
        telemetry.addLine("DPAD ◄ = RED    DPAD ► = BLUE");
        if (autoPose != null) {
            telemetry.addData("Start from Auto pose", "x %.1f  y %.1f  h %.0f°",
                    autoPose.x(), autoPose.y(), Math.toDegrees(autoPose.heading()));
        } else {
            Pose s = startPose();
            telemetry.addData("Start pose", "x %.1f  y %.1f  h %.0f°", s.x(), s.y(), Math.toDegrees(s.heading()));
            telemetry.addLine("Turret must face straight forward now (encoder zeroed).");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        shooter.setAlliance(isRed);
        localizer.setPose(autoPose != null ? autoPose : startPose());
    }

    @Override
    public void loop() {
        localizer.update();
        Pose pose = localizer.pose();

        boolean triangle = gamepad1.triangle;
        if (triangle && !lastTriangle) localizer.setPose(startPose());
        lastTriangle = triangle;

        // ── Drive: always active, field-centric from this alliance's driver station ─
        double driverForward = Math.toRadians(isRed ? RED_DRIVER_FORWARD_DEG : BLUE_DRIVER_FORWARD_DEG);
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,          // Pedro: +strafe = left
                -gamepad1.right_stick_x * 0.7,   // Pedro: +turn = CCW
                pose.heading() - driverForward);
        drivetrain.drive(powers, true);

        // ── Shooter ──────────────────────────────────────────────────────────
        shooter.setIntake(gamepad1.right_trigger - gamepad1.left_trigger);
        if (gamepad1.left_bumper) shooter.shoot();
        extraControls(pose);
        shooter.update(pose, localizer.velocity());

        telemetry.addLine(title());
        telemetry.addData("Alliance", isRed ? "RED" : "BLUE");
        telemetry.addData("Robot", "x %.1f  y %.1f  h %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        shooter.addTelemetry(telemetry);
        extraTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (localizer != null && savePoseOnStop()) Field.savePose(blackboard, localizer.pose(), isRed);
        if (drivetrain != null) drivetrain.stop();
        if (shooter != null) shooter.stop();
    }

    protected Pose startPose() {
        return Field.pose(isRed, RED_START_X, RED_START_Y, RED_START_HEADING_DEG);
    }
}
