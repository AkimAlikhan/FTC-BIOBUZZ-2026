package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.LinkedList;
import java.util.Queue;

/**
 * ─────────────────────────────────────────────────────────────────────────────
 *  RED ALLIANCE TELEOP — ADJUSTABLE LUT TRIM
 *
 *  The LUT still auto-aims (velocity + hood based on distance to goal).
 *  The driver can apply additive offsets on top of the LUT output:
 *
 *  GAMEPAD 1:
 *    Left Stick          → Drive (field-centric)
 *    Right Stick X       → Turn
 *    Right Trigger       → Intake (free) / auto-controlled during shoot
 *    Cross (X)           → Trigger shoot sequence  ← moved from LB
 *
 *    LB  (left  bumper)  → Velocity offset  +25  t/s
 *    LT  (left  trigger) → Velocity offset  −25  t/s  (edge on press)
 *    RB  (right bumper)  → Hood offset      +0.05
 *    RT  (right trigger) → Hood offset      −0.05      (edge on press)
 *
 *    Triangle            → Reset pose to Red start (10, 9)
 *
 *  LIVE PIDF TUNING (hold modifier + DPAD UP/DOWN):
 *    Square       + DPAD → kP ± step
 *    A            + DPAD → kD ± step
 *    B                   → Reset all gains to defaults
 * ─────────────────────────────────────────────────────────────────────────────
 */
@TeleOp(name = "TeleOp: Red Alliance (Adj)", group = "Drive")
public class RedAllianceTeleopAdj extends OpMode {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private DcMotorEx flywheel1, flywheel2, intake, bearing;
    private Servo gate, turnAngle1, turnAngle2;

    // ── Pedro Pathing ─────────────────────────────────────────────────────────
    private Follower follower;

    // ── Red Goal ──────────────────────────────────────────────────────────────
    private static final double RED_GOAL_X = 139.0;
    private static final double RED_GOAL_Y = 139.0;

    // ── Flywheel software PID + feedforward (live-adjustable) ────────────────
    private double flyKF = 0.00039;
    private double flyKP = 0.0022;
    private double flyKI = 0.0;
    private double flyKD = 0.0;

    private static final double STEP_KP = 0.00001;
    private static final double STEP_KD = 0.000001;

    private PIDController flyPID;

    // ── Gate ──────────────────────────────────────────────────────────────────
    private static final double GATE_OPEN  = 0.2;
    private static final double GATE_CLOSE = 0.33;

    // ── Hood (dual servo on one shaft, mirrored) ──────────────────────────────
    private static final boolean MIRROR   = true;
    private static final double  HOOD_MIN = 0.0;
    private static final double  HOOD_MAX = 1.0;
    private double hoodPosition = 0.0;

    // ── Shooter state machine ─────────────────────────────────────────────────
    private enum ShooterState { IDLE, REVING, SHOOT, RESET }
    private ShooterState shooterState = ShooterState.IDLE;
    private ElapsedTime  shooterTimer = new ElapsedTime();
    private boolean      isShooting   = false;

    // ── Flywheel target speed (set by LUT, trimmed by driver) ────────────────
    private double targetVelocity = 0;

    // ── Driver trim offsets (additive on top of LUT output) ──────────────────
    private double velocityOffset = 0.0;   // adjusted ±25 via LB / LT
    private double hoodOffset     = 0.0;   // adjusted ±0.05 via RB / RT

    private static final double VEL_TRIM_STEP  = 25.0;
    private static final double HOOD_TRIM_STEP = 0.05;

    // ── Turret auto-aim constants ─────────────────────────────────────────────
    private static final double TICKS_PER_MOTOR_REV = 383.6;
    private static final double TURRET_GEAR_RATIO    = 125.0 / 33.0;
    private static final double TICKS_PER_DEG        = TICKS_PER_MOTOR_REV * TURRET_GEAR_RATIO / 360.0;
    private static final double TURRET_OFFSET_INCHES = 25.83 / 25.4;
    private static final double TURRET_KP        = 0.065;
    private static final double TURRET_KI        = 0.0;
    private static final double TURRET_KD        = 0.00011;
    private static final double TURRET_TOLERANCE = 0.5;
    private static final double TURRET_MAX_DEG   =  270.0;
    private static final double TURRET_MIN_DEG   = -120.0;
    private static final double TURRET_MAX_POWER =    1.0;
    private static final int    FILTER_SIZE      = 4;

    // ── Lookup table: distance (in) → hood position / flywheel velocity ──────
    private static final double[] LUT_DIST = { 40,   50,   60,   70,   80,   90,   100,  110,  120,  130,  140,  150,  160  };
    private static final double[] LUT_VEL  = { 1000, 1100, 1200, 1200, 1250, 1300, 1400, 1450, 1550, 1600, 1650, 1700, 1750 };
    private static final double[] LUT_HOOD = { 0.1,  0.4,  0.6,  0.6,  0.7,  0.7,  0.8,  0.8,  0.9,  0.9,  1.0,  1.0,  1.0 };

    // ── Turret state ──────────────────────────────────────────────────────────
    private double        lastBearingError = 0;
    private Queue<Double> filterBuffer     = new LinkedList<>();

    // ── Button edge detection ─────────────────────────────────────────────────
    private boolean lastDpadUp   = false;
    private boolean lastDpadDown = false;
    private boolean lastLB       = false;   // velocity trim up
    private boolean lastLT       = false;   // velocity trim down
    private boolean lastRB       = false;   // hood trim up
    private boolean lastRT       = false;   // hood trim down
    private boolean lastCross    = false;   // shoot trigger

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void init() {
        // Drive motors
        leftFront  = hardwareMap.get(DcMotorEx.class, "fl");
        leftRear   = hardwareMap.get(DcMotorEx.class, "bl");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");
        rightRear  = hardwareMap.get(DcMotorEx.class, "br");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Flywheels
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel1.setPower(0);
        flywheel2.setPower(0);
        flyPID = new PIDController(flyKP, flyKI, flyKD);

        // Intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Turret bearing motor
        bearing = hardwareMap.get(DcMotorEx.class, "bearing");
        bearing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bearing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bearing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bearing.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        gate       = hardwareMap.get(Servo.class, "gate");
        turnAngle1 = hardwareMap.get(Servo.class, "turnAngle1");
        turnAngle2 = hardwareMap.get(Servo.class, "turnAngle2");
        gate.setPosition(GATE_CLOSE);
        setHood(hoodPosition);

        // Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        if (blackboard.containsKey("lastX") && blackboard.containsKey("lastY") && blackboard.containsKey("lastHeading")) {
            follower.setStartingPose(new Pose(
                    (double) blackboard.get("lastX"),
                    (double) blackboard.get("lastY"),
                    (double) blackboard.get("lastHeading")
            ));
        } else {
            follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        }

        telemetry.addLine("RED ALLIANCE TELEOP (ADJ) READY");
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.getPose();

        // ── Reset pose ───────────────────────────────────────────────────────
        if (gamepad1.triangle) follower.setPose(new Pose(10, 9, Math.toRadians(180)));

        // ── Field-centric drive ───────────────────────────────────────────────
        driveFieldCentric(
                -gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                -gamepad1.right_stick_x * 0.7,
                pose.getHeading()
        );

        // ── DPAD edge detection (for PIDF tuning) ────────────────────────────
        boolean dpadUp   = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean edgeUp   = dpadUp   && !lastDpadUp;
        boolean edgeDown = dpadDown && !lastDpadDown;
        lastDpadUp   = dpadUp;
        lastDpadDown = dpadDown;

        // ── Bumper/trigger edge detection ────────────────────────────────────
        boolean lb = gamepad1.dpad_up;
        boolean lt = gamepad1.dpad_down;
        boolean rb = gamepad1.dpad_right;
        boolean rt = gamepad1.dpad_left;

        boolean edgeLB = lb && !lastLB;
        boolean edgeLT = lt && !lastLT;
        boolean edgeRB = rb && !lastRB;
        boolean edgeRT = rt && !lastRT;

        lastLB = lb;
        lastLT = lt;
        lastRB = rb;
        lastRT = rt;

        // ── Velocity trim: LB = +25, LT = −25 ────────────────────────────────
        if (edgeLB) velocityOffset += VEL_TRIM_STEP;
        if (edgeLT) velocityOffset -= VEL_TRIM_STEP;

        // ── Hood trim: RB = +0.05, RT = −0.05 ────────────────────────────────
        if (edgeRB) hoodOffset += HOOD_TRIM_STEP;
        if (edgeRT) hoodOffset -= HOOD_TRIM_STEP;

        // ── Live PIDF tuning (modifier + DPAD UP/DOWN) ────────────────────────
        boolean modKP = gamepad1.square;
        boolean modKD = gamepad1.a;

        if (modKP) {
            if (edgeUp)   flyKP = Math.max(0, flyKP + STEP_KP);
            if (edgeDown) flyKP = Math.max(0, flyKP - STEP_KP);
        } else if (modKD) {
            if (edgeUp)   flyKD = Math.max(0, flyKD + STEP_KD);
            if (edgeDown) flyKD = Math.max(0, flyKD - STEP_KD);
        }

        // ── Reset gains (B) + reset trim offsets ─────────────────────────────
        if (gamepad1.b) {
            flyKF = 0.00039;
            flyKP = 0.00333;
            flyKI = 0.0;
            flyKD = 0.0;
            flyPID = new PIDController(flyKP, flyKI, flyKD);
            velocityOffset = 0.0;
            hoodOffset     = 0.0;
        }

        // Apply gains live every loop
        flyPID.setP(flyKP);
        flyPID.setI(flyKI);
        flyPID.setD(flyKD);

        // ── Shoot trigger: Cross (X) ──────────────────────────────────────────
        boolean cross     = gamepad1.cross;
        boolean edgeCross = cross && !lastCross;
        lastCross = cross;

        if (edgeCross && !isShooting) {
            isShooting = true;
            shooterState = ShooterState.REVING;
            shooterTimer.reset();
        }

        // ── Auto-aim turret + set LUT base velocity/hood ──────────────────────
        //   autoAim() writes targetVelocity and hoodPosition from the LUT,
        //   then the trim offsets are applied on top below.
        autoAim(pose);

        // ── Apply driver trim offsets on top of LUT values ───────────────────
        targetVelocity = targetVelocity + velocityOffset;
        hoodPosition   = Math.max(HOOD_MIN, Math.min(HOOD_MAX, hoodPosition + hoodOffset));
        setHood(hoodPosition);

        // ── Flywheel + gate + intake via state machine ─────────────────────────
        runShooterControl();
        updateShooterStateMachine();

        // ── Telemetry ─────────────────────────────────────────────────────────
        double hRad       = pose.getHeading();
        double turretX    = pose.getX() - TURRET_OFFSET_INCHES * Math.cos(hRad);
        double turretY    = pose.getY() - TURRET_OFFSET_INCHES * Math.sin(hRad);
        double distToGoal = Math.hypot(RED_GOAL_X - turretX, RED_GOAL_Y - turretY);
        double turretAngle = bearing.getCurrentPosition() / TICKS_PER_DEG;
        double fw1Vel     = flywheel1.getVelocity();
        double fw2Vel     = flywheel2.getVelocity();
        boolean atSpeed   = isShooting && Math.abs(fw2Vel - targetVelocity) < 80;

        telemetry.addLine("══════  RED ALLIANCE TELEOP (ADJ)  ══════");
        telemetry.addLine("");
        telemetry.addLine("── Shooter ──────────────────────────");
        telemetry.addData("State",      shooterState);
        telemetry.addData("LUT Vel",    "%.0f  t/s  (trim: %+.0f  →  %.0f)", targetVelocity - velocityOffset, velocityOffset, targetVelocity);
        telemetry.addData("FW1 Actual", "%.0f  (err: %+.0f)", fw1Vel, targetVelocity - fw1Vel);
        telemetry.addData("FW2 Actual", "%.0f  (err: %+.0f)", fw2Vel, targetVelocity - fw2Vel);
        telemetry.addData("At speed",   atSpeed ? "✓ YES" : "...");
        telemetry.addData("Gate",       "%.2f", gate.getPosition());
        telemetry.addLine("");
        telemetry.addLine("── Hood ─────────────────────────────");
        telemetry.addData("Hood Pos",   "%.3f  (trim: %+.3f)", hoodPosition, hoodOffset);
        telemetry.addData("Servo 1",    "%.3f", turnAngle1.getPosition());
        telemetry.addData("Servo 2",    "%.3f  (mirrored)", turnAngle2.getPosition());
        telemetry.addLine("");
        telemetry.addLine("── Goal & Turret ────────────────────");
        telemetry.addData("Turret → Red Goal", "%.1f in", distToGoal);
        telemetry.addData("Turret Center",     "X: %.2f  Y: %.2f", turretX, turretY);
        telemetry.addData("Turret Angle",      "%.1f deg", turretAngle);
        telemetry.addData("Robot Pos",         "X: %.1f  Y: %.1f", pose.getX(), pose.getY());
        telemetry.addLine("");
        telemetry.addLine("── PIDF Gains (live) ────────────────");
        telemetry.addData("kF",             "%.5f", flyKF);
        telemetry.addData("kP  (SQ+DPAD)",  "%.5f", flyKP);
        telemetry.addData("kI  (keep = 0)", "%.5f", flyKI);
        telemetry.addData("kD  (A+DPAD)",   "%.6f", flyKD);
        telemetry.addLine("");
        telemetry.addLine("── Controls ─────────────────────────");
        telemetry.addLine("CROSS           → Shoot");
        telemetry.addLine("LB / LT         → Vel trim  +25 / −25");
        telemetry.addLine("RB / RT         → Hood trim +0.05 / −0.05");
        telemetry.addLine("B               → Reset gains + trim");
        telemetry.addLine("R.TRIG          → Intake (free)");
        telemetry.addLine("TRIANGLE        → Reset pose");
        telemetry.addLine("SQ+DPAD         → kP  |  A+DPAD → kD");
        telemetry.update();
    }

    // ── Flywheel power via software PID (called every loop) ──────────────────
    private void runShooterControl() {
        if (shooterState == ShooterState.IDLE) {
            flyPID.reset();
            flywheel1.setPower(0);
            flywheel2.setPower(0);
            return;
        }
        double avgVel = (flywheel1.getVelocity() + flywheel2.getVelocity()) / 2.0;
        double ff     = flyKF * targetVelocity;
        double pid    = flyPID.calculate(avgVel, targetVelocity);
        double pwr    = Math.max(-1.0, Math.min(1.0, ff + pid));
        flywheel1.setPower(pwr);
        flywheel2.setPower(pwr);
    }

    // ── Shooter state machine ─────────────────────────────────────────────────
    private void updateShooterStateMachine() {
        if (!isShooting) {
            // Free intake on right trigger when not shooting
            intake.setPower(-gamepad1.right_trigger);
            return;
        }

        double currentVel = Math.max(flywheel2.getVelocity(), flywheel1.getVelocity());

        switch (shooterState) {
            case REVING:
                intake.setPower(0);
                if (Math.abs(currentVel - targetVelocity) < 50 || shooterTimer.seconds() > 2) {
                    shooterState = ShooterState.SHOOT;
                    shooterTimer.reset();
                }
                break;

            case SHOOT:
                gate.setPosition(GATE_OPEN);
                intake.setPower(-1);
                if (shooterTimer.seconds() > 1) {
                    shooterState = ShooterState.RESET;
                    shooterTimer.reset();
                }
                break;

            case RESET:
                gate.setPosition(GATE_CLOSE);
                intake.setPower(0);
                isShooting   = false;
                shooterState = ShooterState.IDLE;
                break;
        }
    }

    // ── Auto-aim turret toward Red Goal + write LUT base values ──────────────
    //   NOTE: this sets targetVelocity and hoodPosition from the LUT.
    //         Driver trim offsets are applied in loop() AFTER this returns.
    private void autoAim(Pose robotPose) {
        double hRad    = robotPose.getHeading();
        double turretX = robotPose.getX() - TURRET_OFFSET_INCHES * Math.cos(hRad);
        double turretY = robotPose.getY() - TURRET_OFFSET_INCHES * Math.sin(hRad);

        double dx       = Math.abs(RED_GOAL_X - turretX);
        double dy       = Math.abs(RED_GOAL_Y - turretY);
        double distance = Math.sqrt(dx * dx + dy * dy);

        // LUT base values (trim applied later in loop())
        hoodPosition   = 0;
        targetVelocity = 0;

        double angleToGoalGlobal = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg   = wrapDeg(Math.toDegrees(hRad));
        double rawDesiredAngle   = -wrapDeg2(angleToGoalGlobal - robotHeadingDeg);

        double currentAngle = bearing.getCurrentPosition() / TICKS_PER_DEG;
        double bestTarget   = rawDesiredAngle;
        double bestDist     = Double.MAX_VALUE;
        for (int n = -1; n <= 1; n++) {
            double candidate = rawDesiredAngle + n * 360.0;
            if (candidate >= TURRET_MIN_DEG && candidate <= TURRET_MAX_DEG) {
                double dist = Math.abs(candidate - currentAngle);
                if (dist < bestDist) { bestDist = dist; bestTarget = candidate; }
            }
        }

        filterBuffer.add(bestTarget);
        if (filterBuffer.size() > FILTER_SIZE) filterBuffer.remove();
        double smoothed = 0;
        for (double v : filterBuffer) smoothed += v;
        smoothed /= filterBuffer.size();

        double constrained = Math.max(TURRET_MIN_DEG, Math.min(TURRET_MAX_DEG, smoothed));
        double error       = constrained - currentAngle;

        if (Math.abs(error) < TURRET_TOLERANCE) {
            bearing.setPower(0);
            lastBearingError = error;
            return;
        }

        double power = error * TURRET_KP + (error - lastBearingError) * TURRET_KD;
        power = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, power));
        lastBearingError = error;

        if (currentAngle > TURRET_MAX_DEG && power > 0) power = 0;
        else if (currentAngle < TURRET_MIN_DEG && power < 0) power = 0;

        bearing.setPower(power);
    }

    // ── Commands both hood servos together ────────────────────────────────────
    private void setHood(double pos) {
        turnAngle1.setPosition(pos);
        turnAngle2.setPosition(MIRROR ? 1.0 - pos : pos);
    }

    private void driveFieldCentric(double strafe, double forward, double turn, double headingRad) {
        double rotX = strafe  * Math.cos(-headingRad) - (-forward) * Math.sin(-headingRad);
        double rotY = strafe  * Math.sin(-headingRad) + (-forward) * Math.cos(-headingRad);
        double fl = rotY + rotX + turn;
        double bl = rotY - rotX + turn;
        double fr = rotY - rotX - turn;
        double br = rotY + rotX - turn;
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        leftFront.setPower(-fl / max);
        leftRear.setPower(-bl / max);
        rightFront.setPower(-fr / max);
        rightRear.setPower(-br / max);
    }

    private double wrapDeg(double deg) {
        double d = deg % 360.0;
        if (d < 0) d += 360.0;
        return d;
    }

    private double wrapDeg2(double deg) {
        double d = deg % 360.0;
        if (d < -180) d += 360.0;
        if (d >= 180) d -= 360.0;
        return d;
    }

    private double lutInterp(double[] table, double dist) {
        if (dist <= LUT_DIST[0])                    return table[0];
        if (dist >= LUT_DIST[LUT_DIST.length - 1]) return table[table.length - 1];
        int i = 0;
        while (i < LUT_DIST.length - 1 && LUT_DIST[i + 1] < dist) i++;
        double t = (dist - LUT_DIST[i]) / (LUT_DIST[i + 1] - LUT_DIST[i]);
        return table[i] + t * (table[i + 1] - table[i]);
    }
}
