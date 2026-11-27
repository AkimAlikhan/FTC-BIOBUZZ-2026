package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
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
 * Call update(pose, velocity) every loop. It:
 *   - picks the hive target (LOW if robot y < 72, HIGH otherwise; blue is point-mirrored),
 *   - leads the target for robot motion (shoot on the move),
 *   - aims the turret,
 *   - runs the shot sequence: REVING -> GATE_WAIT -> SHOOT -> RESET -> IDLE.
 *
 * shoot() starts a shot. While IDLE, the intake runs at setIntake(power).
 */
public class Shooter {

    // ── Hive targets, written for RED. Blue = mirrored through the field center. ──
    public static double RED_TARGET_X      = 58;
    public static double RED_TARGET_LOW_Y  = 55;
    public static double RED_TARGET_HIGH_Y = 89;
    public static double SPLIT_Y           = 72;

    // ── Flywheel (1 motor): power = kF * target + kP * (target - velocity) ───
    public static double FLY_KF = 0.00039;
    public static double FLY_KP = 0.0022;
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
    public static double HOOD_POS   = 0.5;  // fixed for now

    // ── Turret ───────────────────────────────────────────────────────────────
    private static final double TICKS_PER_MOTOR_REV  = 383.6;
    private static final double TURRET_GEAR_RATIO    = 125.0 / 33.0;
    private static final double TICKS_PER_DEG        = TICKS_PER_MOTOR_REV * TURRET_GEAR_RATIO / 360.0;
    private static final double TURRET_OFFSET_INCHES = 25.83 / 25.4; // turret center is behind robot center
    private static final double TURRET_KP        = 0.065;
    private static final double TURRET_KD        = 0.00011;
    private static final double TURRET_TOLERANCE = 0.5;   // deg
    private static final double TURRET_MAX_DEG   = 180;
    private static final double TURRET_MIN_DEG   = -180;
    private static final double TURRET_MAX_POWER = 1.0;
    private static final int    FILTER_SIZE      = 4;

    // ── LUT: distance turret→target (in) → flywheel velocity / time of flight ─
    private static final double[] LUT_DIST = { 40,    50,    60,    70,    80,    90,    100,   110,   120,   130,   140,   150,   160,   170   };
    private static final double[] LUT_VEL  = { 1025,  1050,  1125,  1225,  1250,  1350,  1375,  1425,  1500,  1525,  1600,  1650,  1725,  1800  };
    private static final double[] LUT_TOF  = { 0.800, 0.600, 0.530, 0.583, 0.635, 0.585, 0.670, 0.665, 0.670, 0.705, 0.740, 0.735, 0.800, 0.880 };

    public enum State { IDLE, REVING, GATE_WAIT, SHOOT, RESET }

    private final DcMotorEx flywheel, intakeLeft, intakeRight, bearing;
    private final Servo hood, gate;

    private boolean isRed = true;
    private State state = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private double manualIntake = 0;

    private double lastTurretError = 0;
    private final ArrayDeque<Double> turretFilter = new ArrayDeque<>();

    // Last-loop values, for telemetry.
    private double targetX, targetY, realDist, virtDist, virtX, virtY, tof;
    private double targetVel, flyVel, flyPower, turretAngle;

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

    public void update(Pose pose, Velocity fieldVelocity) {
        // ── Target: point-mirror the robot into red coordinates, choose, mirror back ─
        double redFrameY = isRed ? pose.y() : 144 - pose.y();
        double redY = redFrameY < SPLIT_Y ? RED_TARGET_LOW_Y : RED_TARGET_HIGH_Y;
        targetX = isRed ? RED_TARGET_X : 144 - RED_TARGET_X;
        targetY = isRed ? redY : 144 - redY;

        // ── Shoot on the move: aim at a target shifted against our velocity ──
        double h = pose.heading();
        double turretX = pose.x() - TURRET_OFFSET_INCHES * Math.cos(h);
        double turretY = pose.y() - TURRET_OFFSET_INCHES * Math.sin(h);
        realDist = Math.hypot(targetX - turretX, targetY - turretY);

        tof = lut(LUT_TOF, realDist);
        virtX = targetX - fieldVelocity.vx * tof;
        virtY = targetY - fieldVelocity.vy * tof;
        virtDist = Math.hypot(virtX - turretX, virtY - turretY);
        targetVel = lut(LUT_VEL, virtDist);

        hood.setPosition(HOOD_POS);
        turretAngle = aimTurret(Math.toDegrees(Math.atan2(virtY - turretY, virtX - turretX)), Math.toDegrees(h));

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

        flyPower = state == State.IDLE ? 0 : clip(FLY_KF * targetVel + FLY_KP * (targetVel - flyVel), 0, 1);
        flywheel.setPower(flyPower);
    }

    public void addTelemetry(Telemetry t) {
        t.addData("Target", "(%.0f, %.0f)", targetX, targetY);
        t.addData("Dist real / virt", "%.1f / %.1f in", realDist, virtDist);
        t.addData("Lead", "x %.1f  y %.1f  (ToF %.2f s)", targetX - virtX, targetY - virtY, tof);
        t.addData("Turret", "%.1f°", turretAngle);
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

    private static double lut(double[] table, double dist) {
        if (dist <= LUT_DIST[0]) return table[0];
        int last = LUT_DIST.length - 1;
        if (dist >= LUT_DIST[last]) return table[last];
        int i = 0;
        while (LUT_DIST[i + 1] < dist) i++;
        double t = (dist - LUT_DIST[i]) / (LUT_DIST[i + 1] - LUT_DIST[i]);
        return table[i] + t * (table[i + 1] - table[i]);
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
