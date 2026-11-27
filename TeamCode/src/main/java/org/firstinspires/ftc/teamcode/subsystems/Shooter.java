package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayDeque;

/**
 * Flywheel + intake + gate + hood + turret, shared by TeleOp and Auto.
 *
 * Call update(pose) every loop. It:
 *   - picks the hive target (LOW if robot y < 72, HIGH otherwise; blue is point-mirrored),
 *   - aims the turret,
 *   - runs the shot sequence: REVING -> GATE_WAIT -> SHOOT -> RESET -> IDLE.
 *
 * shoot() starts a shot. While IDLE, the intake runs at setIntake(power).
 */
public class Shooter {

    // ── Hive targets, written for RED. Blue = mirrored through the field center
    // (x -> 144 - x, y -> 144 - y). LOW is used when the robot is below SPLIT_Y, HIGH above.
    public static double RED_LOW_X  = 60, RED_LOW_Y  = 54;
    public static double RED_HIGH_X = 60, RED_HIGH_Y = 144 - 54;  // = (60, 90)
    public static double SPLIT_Y           = 72;

    // ── Flywheel (1 motor). Gains live in FlywheelPIDF (tune with "TUNE: Flywheel PIDF"). ─
    public static double AT_SPEED_TOLERANCE = 50; // ticks/s
    public static DcMotorSimple.Direction FLYWHEEL_DIR = DcMotorSimple.Direction.REVERSE;

    // ── Shot timing (2025-26 values) ─────────────────────────────────────────
    public static double REV_TIMEOUT_SEC  = 2.0;  // max wait for flywheel to reach speed
    public static double GATE_WAIT_SEC    = 0.5;  // gate open, before feeding
    public static double SHOOT_SEC        = 1.0;  // intake feeds for this long
    public static double LONG_SHOT_DIST   = 120;  // beyond this, feed a bit slower
    public static double LONG_SHOT_INTAKE = 0.85;

    // ── Intake (2 motors, same direction) ────────────────────────────────────
    public static DcMotorSimple.Direction INTAKE_LEFT_DIR  = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction INTAKE_RIGHT_DIR = DcMotorSimple.Direction.FORWARD;

    // ── Gate + hood ──────────────────────────────────────────────────────────
    public static double GATE_OPEN  = 0.91;
    public static double GATE_CLOSE = 0.75;
    public static double HOOD_POS   = 0.5;  // used while USE_HOOD_LUT is false
    public static boolean USE_HOOD_LUT = false; // true = hood from LUT_HOOD by distance

    // ── Turret ───────────────────────────────────────────────────────────────
    private static final double TICKS_PER_MOTOR_REV  = 383.6;
    private static final double TURRET_GEAR_RATIO    = 125.0 / 33.0;
    private static final double TICKS_PER_DEG        = TICKS_PER_MOTOR_REV * TURRET_GEAR_RATIO / 360.0;
    private static final double TURRET_OFFSET_INCHES = 25.83 / 25.4; // turret center is behind robot center
    public static double TURRET_KP = 0.04;  // was 0.065; lower = gentler turret, higher = snappier
    private static final double TURRET_KD        = 0.00011;
    private static final double TURRET_TOLERANCE = 0.5;   // deg
    private static final double TURRET_MAX_DEG   = 180;
    private static final double TURRET_MIN_DEG   = -180;
    private static final double TURRET_MAX_POWER = 1.0;
    private static final int    FILTER_SIZE      = 4;

    // ── LUT: distance turret→target (in) → flywheel velocity / time of flight ─
    // Fill these from "TUNE: LUT Finder". LUT_DIST must be increasing; all rows the same length.
    public static double[] LUT_DIST = { 40,    50,    60,    70,    80,    90,    100,   110,   120,   130,   140,   150,   160,   170   };
    public static double[] LUT_VEL  = { 1025,  1050,  1125,  1225,  1250,  1350,  1375,  1425,  1500,  1525,  1600,  1650,  1725,  1800  };
    public static double[] LUT_HOOD = { 0.15,  0.4,   0.45,  0.7,   0.7,   0.75,  0.75,  0.8,   0.8,   0.8,   0.85,  0.85,  0.9,   0.9   };

    public enum State { IDLE, REVING, GATE_WAIT, SHOOT, RESET }

    private final DcMotorEx flywheel, intakeLeft, intakeRight, bearing;
    private final Servo hood, gate;

    private boolean isRed = true;
    private State state = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private double manualIntake = 0;
    private final FlywheelPIDF flywheelPIDF = new FlywheelPIDF();

    // Manual overrides (LUT Finder). null = use the LUT.
    private Double velOverride = null, hoodOverride = null;

    private double lastTurretError = 0;
    private final ArrayDeque<Double> turretFilter = new ArrayDeque<>();

    // Last-loop values, for telemetry.
    private boolean aimingHigh;
    private double targetX, targetY, realDist;
    private double targetVel, flyVel, flyPower, turretAngle, hoodPos;

    /**
     * @param zeroTurret true = the turret faces straight forward right now, reset its encoder.
     *                   false = keep the encoder count (e.g. TeleOp right after Auto).
     */
    public Shooter(HardwareMap hw, boolean zeroTurret) {
        flywheel = hw.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(FLYWHEEL_DIR); // also flips getVelocity(), so the speed loop stays positive
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setPower(0);

        intakeLeft  = hw.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hw.get(DcMotorEx.class, "intakeRight");
        intakeLeft.setDirection(INTAKE_LEFT_DIR);
        intakeRight.setDirection(INTAKE_RIGHT_DIR);

        bearing = hw.get(DcMotorEx.class, "bearing");
        if (zeroTurret) bearing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bearing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bearing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bearing.setDirection(DcMotorSimple.Direction.REVERSE);

        hood = hw.get(Servo.class, "turnAngle");
        gate = hw.tryGet(Servo.class, "gate");
        hood.setPosition(HOOD_POS);
        setGate(GATE_CLOSE);
    }

    public void setAlliance(boolean red) { isRed = red; }

    /** Starts a shot if one isn't already running. */
    public void shoot() {
        if (state == State.IDLE) setState(State.REVING);
    }

    public boolean isBusy() { return state != State.IDLE; }

    /** Use these instead of the LUT (pass null to go back to the LUT). For finding LUT values. */
    public void setOverride(Double flywheelVel, Double hood) {
        velOverride = flywheelVel;
        hoodOverride = hood;
    }

    /** Distance from the turret to the real (not lead-corrected) target, last loop. */
    public double distance() { return realDist; }
    /** The hive cell being aimed at (field coordinates). */
    public double targetX() { return targetX; }
    public double targetY() { return targetY; }
    /** true = robot is above the split line (y >= 72 in red terms), so it aims at the HIGH cell. */
    public boolean aimingHigh() { return aimingHigh; }
    public double targetVelocity() { return targetVel; }
    public double flywheelVelocity() { return flyVel; }
    public double hoodPosition() { return hoodPos; }

    public State state() { return state; }

    /** Intake power used while not shooting (+ = in). */
    public void setIntake(double power) { manualIntake = power; }

    /** Turns everything off and cancels any shot. The turret holds still. */
    public void stop() {
        setState(State.IDLE);
        manualIntake = 0;
        flywheel.setPower(0);
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        bearing.setPower(0);
        setGate(GATE_CLOSE);
    }

    public void update(Pose pose) {
        // ── Target: point-mirror the robot into red coordinates, choose, mirror back ─
        double redFrameY = isRed ? pose.y() : 144 - pose.y();
        aimingHigh = redFrameY >= SPLIT_Y;
        double redX = aimingHigh ? RED_HIGH_X : RED_LOW_X;
        double redY = aimingHigh ? RED_HIGH_Y : RED_LOW_Y;
        targetX = isRed ? redX : 144 - redX;
        targetY = isRed ? redY : 144 - redY;

        // ── Distance + aim from the turret center straight at the hive cell ──
        double h = pose.heading();
        double turretX = pose.x() - TURRET_OFFSET_INCHES * Math.cos(h);
        double turretY = pose.y() - TURRET_OFFSET_INCHES * Math.sin(h);
        realDist = Math.hypot(targetX - turretX, targetY - turretY);

        targetVel = velOverride != null ? velOverride : lut(LUT_DIST, LUT_VEL, realDist);
        hoodPos = hoodOverride != null ? hoodOverride : (USE_HOOD_LUT ? lut(LUT_DIST, LUT_HOOD, realDist) : HOOD_POS);
        hood.setPosition(hoodPos);
        turretAngle = aimTurret(Math.toDegrees(Math.atan2(targetY - turretY, targetX - turretX)), Math.toDegrees(h));

        // ── Shot sequence ────────────────────────────────────────────────────
        flyVel = flywheel.getVelocity();
        double intake = 0;
        switch (state) {
            case IDLE:
                intake = manualIntake;
                break;
            case REVING:
                if (Math.abs(targetVel - flyVel) < AT_SPEED_TOLERANCE || timer.seconds() > REV_TIMEOUT_SEC) {
                    setState(State.GATE_WAIT);
                }
                break;
            case GATE_WAIT:
                setGate(GATE_OPEN);
                if (timer.seconds() > GATE_WAIT_SEC) setState(State.SHOOT);
                break;
            case SHOOT:
                intake = realDist > LONG_SHOT_DIST ? LONG_SHOT_INTAKE : 1.0;
                if (timer.seconds() > SHOOT_SEC) setState(State.RESET);
                break;
            case RESET:
                // close gate, stop flywheel, stop intake
                setGate(GATE_CLOSE);
                manualIntake = 0;
                setState(State.IDLE);
                break;
        }
        intakeLeft.setPower(intake);
        intakeRight.setPower(intake);

        if (state == State.IDLE) {
            flywheelPIDF.reset();
            flyPower = 0;
        } else {
            flyPower = flywheelPIDF.calculate(targetVel, flyVel);
        }
        flywheel.setPower(flyPower);
    }

    public void addTelemetry(Telemetry t) {
        t.addData("Target", "(%.0f, %.0f)", targetX, targetY);
        t.addData("Distance", "%.1f in", realDist);
        t.addData("Turret", "%.1f°", turretAngle);
        t.addData("Hood", "%.2f%s", hoodPos, hoodOverride != null ? " (manual)" : "");
        t.addData("Shooter", state);
        t.addData("Flywheel", "target %.0f  actual %.0f  pwr %.2f", targetVel, flyVel, flyPower);
    }

    private void setState(State s) {
        state = s;
        timer.reset();
    }

    private void setGate(double pos) {
        if (gate != null) gate.setPosition(pos);
    }

    /** Turns the turret toward the given field angle. Returns the current turret angle (deg). */
    private double aimTurret(double angleToTargetDeg, double robotHeadingDeg) {
        double raw = -wrap180(angleToTargetDeg - robotHeadingDeg);
        double current = bearing.getCurrentPosition() / TICKS_PER_DEG;

        // Choose raw, raw+360 or raw-360: the one inside the turret limits closest to where we are.
        double best = raw, bestDist = Double.MAX_VALUE;
        for (int n = -1; n <= 1; n++) {
            double c = raw + n * 360.0;
            if (c >= TURRET_MIN_DEG && c <= TURRET_MAX_DEG && Math.abs(c - current) < bestDist) {
                bestDist = Math.abs(c - current);
                best = c;
            }
        }

        turretFilter.addLast(best);
        if (turretFilter.size() > FILTER_SIZE) turretFilter.removeFirst();
        double smoothed = 0;
        for (double v : turretFilter) smoothed += v;
        smoothed /= turretFilter.size();

        double target = clip(smoothed, TURRET_MIN_DEG, TURRET_MAX_DEG);
        double error = target - current;
        if (Math.abs(error) < TURRET_TOLERANCE) {
            bearing.setPower(0);
            lastTurretError = error;
            return current;
        }

        double power = clip(error * TURRET_KP + (error - lastTurretError) * TURRET_KD,
                -TURRET_MAX_POWER, TURRET_MAX_POWER);
        lastTurretError = error;
        if ((current > TURRET_MAX_DEG && power > 0) || (current < TURRET_MIN_DEG && power < 0)) power = 0;
        bearing.setPower(power);
        return current;
    }

    /** Linear interpolation of ys over xs (xs increasing, same length). Clamped at both ends. */
    public static double lut(double[] xs, double[] ys, double x) {
        int n = Math.min(xs.length, ys.length);
        if (n == 0) return 0;
        if (x <= xs[0]) return ys[0];
        if (x >= xs[n - 1]) return ys[n - 1];
        int i = 0;
        while (xs[i + 1] < x) i++;
        double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
        return ys[i] + t * (ys[i + 1] - ys[i]);
    }

    private static double wrap180(double deg) {
        double d = deg % 360.0;
        if (d < -180) d += 360.0;
        if (d >= 180) d -= 360.0;
        return d;
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
