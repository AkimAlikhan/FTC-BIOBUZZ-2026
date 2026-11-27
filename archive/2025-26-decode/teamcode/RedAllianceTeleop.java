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
import org.firstinspires.ftc.teamcode.LimelightLocalizer;

import java.util.LinkedList;
import java.util.Queue;

/**
 * ─────────────────────────────────────────────────────────────────────────────
 *  RED ALLIANCE TELEOP
 *
 *  GAMEPAD 1 (all controls):
 *    Left Stick          → Drive (field-centric)
 *    Right Stick X       → Turn
 *    Right Trigger       → Intake (free) / auto-controlled during shoot
 *    Left Bumper         → Trigger shoot sequence
 *    Right Bumper        → Hood position + 0.1
 *    Y                   → Hood position − 0.1
 *    DPAD UP / DOWN      → Flywheel target speed ± 50 ticks/sec
 *    Triangle            → Reset pose to Red start (10, 9)
 *
 *  LIVE PIDF TUNING (hold modifier + DPAD UP/DOWN):
 *    Left Trigger + DPAD → kF ± step
 *    Square       + DPAD → kP ± step
 *    A            + DPAD → kD ± step
 *    B                   → Reset all gains to defaults
 * ─────────────────────────────────────────────────────────────────────────────
 */
@TeleOp(name = "TeleOp: Red Alliance", group = "Drive")
public class RedAllianceTeleop extends OpMode {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private DcMotorEx flywheel1, flywheel2, intake, bearing;
    private Servo gate, turnAngle1, turnAngle2;

    // ── Pedro Pathing ─────────────────────────────────────────────────────────
    private Follower follower;

    // ── Limelight AprilTag Localizer ──────────────────────────────────────────
    private LimelightLocalizer limelightLocalizer;

    // ── Red Goal ──────────────────────────────────────────────────────────────
    private static final double RED_GOAL_X = 139.0;
    private static final double RED_GOAL_Y = 139.0;

    // ── Flywheel software PID + feedforward (live-adjustable) ────────────────
    //
    //  power = kF × targetVel  +  PID(currentVel, targetVel)
    //
    //  ┌──────────────────────────────────────────────────────────────────────┐
    //  │  TUNING GUIDE  (tune on the robot using GP1 modifier + DPAD)        │
    //  ├──────────────────────────────────────────────────────────────────────┤
    //  │  GoBilda 6000 RPM  |  28 ticks/rev  |  max ≈ 2800 ticks/sec        │
    //  │  STEP 1 — kF first. Hold L.TRIG + DPAD. Rev flywheels with LB,     │
    //  │           raise until velocity reaches target within ~150 t/s.      │
    //  │  STEP 2 — kP next. Hold SQ + DPAD. Close the remaining gap.        │
    //  │  STEP 3 — kD only if oscillation remains. Hold A + DPAD.           │
    //  │  Press B to reset all gains to defaults.                            │
    //  └──────────────────────────────────────────────────────────────────────┘

    // Mutable gains — adjusted live via gamepad
    private double flyKF = 0.00039;
    private double flyKP = 0.0022;
    private double flyKI = 0.0;
    private double flyKD = 0.0;

    private static final double STEP_KF = 0.00001;
    private static final double STEP_KP = 0.00001;
    private static final double STEP_KD = 0.000001;

    private PIDController flyPID;

    // ── Gate ──────────────────────────────────────────────────────────────────
    private static final double GATE_OPEN  = 0.2;
    private static final double GATE_CLOSE = 0.33;

    // ── Hood (dual servo on one shaft, mirrored) ──────────────────────────────
    private static final boolean MIRROR    = true;
    private static final double  HOOD_MIN  = 0.0;
    private static final double  HOOD_MAX  = 1.0;
    private static final double  HOOD_STEP = 0.1;
    private double hoodPosition = 0.0;

    // ── Shooter state machine ─────────────────────────────────────────────────
    private enum ShooterState { IDLE, REVING, GATE_WAIT, SHOOT, RESET }
    private ShooterState shooterState = ShooterState.IDLE;
    private ElapsedTime  shooterTimer = new ElapsedTime();
    private boolean      isShooting   = false;

    // ── Flywheel target speed (adjustable via DPAD) ───────────────────────────
    private double targetVelocity = 1300.0;

    // ── Turret auto-aim constants ─────────────────────────────────────────────
    private static final double TICKS_PER_MOTOR_REV = 383.6;
    private static final double TURRET_GEAR_RATIO    = 125.0 / 33.0;
    private static final double TICKS_PER_DEG        = TICKS_PER_MOTOR_REV * TURRET_GEAR_RATIO / 360.0;
    private static final double TURRET_OFFSET_INCHES = 25.83 / 25.4;
    private static final double TURRET_KP        = 0.065;
    private static final double TURRET_KI        = 0.0;
    private static final double TURRET_KD        = 0.00011;
    private static final double TURRET_TOLERANCE = 0.5;  // deg — lower = tighter aim
    private static final double TURRET_MAX_DEG   =  270.0;
    private static final double TURRET_MIN_DEG   = -120.0;
    private static final double TURRET_MAX_POWER =    1;  // cap — less aggressive auto-aim
    private static final int    FILTER_SIZE      = 4;

    // ── Lookup table: distance (in) → hood position / flywheel velocity ──────
    private static final double[] LUT_DIST = { 40,    50,    60,    70,    80,    90,    100,   110,   120,   130,   140,   150,   160,   170   };
    private static final double[] LUT_VEL  = { 1025,  1050,  1125,  1225,  1250,  1350,  1375,  1425,  1500,  1525,  1600,  1650,  1725,  1800  };
    private static final double[] LUT_HOOD = { 0.15,  0.4,   0.45,  0.7,   0.7,   0.75,  0.75,  0.8,   0.8,   0.8,   0.85,  0.85,  0.9,   0.9   };

    // ── Turret state ──────────────────────────────────────────────────────────
    private double        lastBearingError = 0;
    private Queue<Double> filterBuffer     = new LinkedList<>();

    // ── Button edge detection ─────────────────────────────────────────────────
    private boolean lastDpadUp   = false;
    private boolean lastDpadDown = false;
    private boolean lastRB       = false;
    private boolean lastY        = false;

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

        // Flywheels — RUN_WITHOUT_ENCODER; state machine switches to PID when revving
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

        // Limelight Localizer
        limelightLocalizer = new LimelightLocalizer(hardwareMap);

        telemetry.addLine("RED ALLIANCE TELEOP READY");
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void stop() {
        if (limelightLocalizer != null) limelightLocalizer.stop();
    }

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void loop() {
        // ── Limelight Localization (runs before follower.update so Pedro sees corrected pose) ──
        boolean llFix = limelightLocalizer.update(follower);

        follower.update();
        Pose pose = follower.getPose();

        // ── Reset pose ───────────────────────────────────────────────────────
        if (gamepad1.triangle) follower.setPose(new Pose(10, 9, Math.toRadians(180)));

        // ── Field-centric drive ───────────────────────────────────────────────
        driveFieldCentric(
                -gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                -gamepad1.right_stick_x*0.7,
                pose.getHeading()
        );

        // ── Hood: RB = +0.1, Y = −0.1 (edge-triggered) ───────────────────────
        boolean rb = gamepad1.right_bumper;
        boolean y  = gamepad1.cross;
        if (rb && !lastRB) hoodPosition = Math.min(HOOD_MAX, hoodPosition + HOOD_STEP);
        if (y  && !lastY)  hoodPosition = Math.max(HOOD_MIN, hoodPosition - HOOD_STEP);
        lastRB = rb;
        lastY  = y;
        setHood(hoodPosition);

        // ── DPAD edge detection ───────────────────────────────────────────────
        boolean dpadUp   = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean edgeUp   = dpadUp   && !lastDpadUp;
        boolean edgeDown = dpadDown && !lastDpadDown;
        lastDpadUp   = dpadUp;
        lastDpadDown = dpadDown;

        // ── Live PIDF tuning (modifier + DPAD UP/DOWN) ────────────────────────
        boolean modKF = gamepad1.left_trigger > 0.5;
        boolean modKP = gamepad1.square;
        boolean modKD = gamepad1.a;

        if (modKF) {
            if (edgeUp)   flyKF = Math.max(0, flyKF + STEP_KF);
            if (edgeDown) flyKF = Math.max(0, flyKF - STEP_KF);
        } else if (modKP) {
            if (edgeUp)   flyKP = Math.max(0, flyKP + STEP_KP);
            if (edgeDown) flyKP = Math.max(0, flyKP - STEP_KP);
        } else if (modKD) {
            if (edgeUp)   flyKD = Math.max(0, flyKD + STEP_KD);
            if (edgeDown) flyKD = Math.max(0, flyKD - STEP_KD);
        } else {
            // ── Flywheel target speed (DPAD UP / DOWN, no modifier) ───────────
            if (edgeUp)   targetVelocity += 50;
            if (edgeDown) targetVelocity = Math.max(0, targetVelocity - 50);
        }

        // ── Reset gains (B) ───────────────────────────────────────────────────
        if (gamepad1.b) {
            flyKF = 0.00039;
            flyKP = 0.00333;
            flyKI = 0.0;
            flyKD = 0.0;
            flyPID = new PIDController(flyKP, flyKI, flyKD);
        }

        // Apply gains live every loop
        flyPID.setP(flyKP);
        flyPID.setI(flyKI);
        flyPID.setD(flyKD);

        // ── Shoot trigger (LB) ────────────────────────────────────────────────
        if (gamepad1.left_bumper && !isShooting) {
            isShooting = true;
            shooterState = ShooterState.REVING;
            shooterTimer.reset();
        }

        // ── Flywheel + gate + intake via state machine ─────────────────────────
        runShooterControl();
        updateShooterStateMachine();

        // ── Auto-aim turret ───────────────────────────────────────────────────
        autoAim(pose);

        // ── Telemetry ─────────────────────────────────────────────────────────
        double hRad        = pose.getHeading();
        double turretX     = pose.getX() - TURRET_OFFSET_INCHES * Math.cos(hRad);
        double turretY     = pose.getY() - TURRET_OFFSET_INCHES * Math.sin(hRad);
        double distToGoal  = Math.hypot(RED_GOAL_X - turretX, RED_GOAL_Y - turretY);
        double turretAngle = bearing.getCurrentPosition() / TICKS_PER_DEG;
        double fw1Vel      = flywheel1.getVelocity();
        double fw2Vel      = flywheel2.getVelocity();
        boolean atSpeed    = isShooting && Math.abs(fw2Vel - targetVelocity) < 80;

        telemetry.addLine("══════  RED ALLIANCE TELEOP  ══════");
        telemetry.addLine("");
        telemetry.addLine("── Shooter ──────────────────────────");
        telemetry.addData("State",      shooterState);
        telemetry.addData("Target vel", "%.0f  t/s", targetVelocity);
        telemetry.addData("FW1 Actual", "%.0f  (err: %+.0f)", fw1Vel, targetVelocity - fw1Vel);
        telemetry.addData("FW2 Actual", "%.0f  (err: %+.0f)", fw2Vel, targetVelocity - fw2Vel);
        telemetry.addData("At speed",   atSpeed ? "✓ YES" : "...");
        telemetry.addData("Gate",       "%.2f", gate.getPosition());
        telemetry.addLine("");
        telemetry.addLine("── Hood ─────────────────────────────");
        telemetry.addData("Hood Pos",   "%.2f  (Y=−  RB=+)", hoodPosition);
        telemetry.addData("Servo 1",    "%.3f", turnAngle1.getPosition());
        telemetry.addData("Servo 2",    "%.3f  (mirrored)",  turnAngle2.getPosition());
        telemetry.addLine("");
        telemetry.addLine("── Goal & Turret ────────────────────");
        telemetry.addData("Turret → Red Goal", "%.1f in", distToGoal);
        telemetry.addData("Turret Center",     "X: %.2f  Y: %.2f", turretX, turretY);
        telemetry.addData("Turret Angle",      "%.1f deg", turretAngle);
        telemetry.addData("Robot Pos",         "X: %.1f  Y: %.1f", pose.getX(), pose.getY());
        telemetry.addData("LL Fix",            llFix ? "✓ ACTIVE (fixes: " + limelightLocalizer.getFixCount() + ")" : "✗ no fix");
        if (limelightLocalizer.isLastFixValid())
            telemetry.addData("LL Last Hdg",   "%.1f°", limelightLocalizer.getLastHeadingDeg());
        telemetry.addData("LL Debug",          limelightLocalizer.getDebugInfo());
        telemetry.addLine("");
        telemetry.addLine("── PIDF Gains (live) ────────────────");
        telemetry.addData("kF  (L.TRIG+DPAD)", "%.5f", flyKF);
        telemetry.addData("kP  (SQ+DPAD)",     "%.5f", flyKP);
        telemetry.addData("kI  (keep = 0)",    "%.5f", flyKI);
        telemetry.addData("kD  (A+DPAD)",      "%.6f", flyKD);
        telemetry.addLine("");
        telemetry.addLine("── Controls ─────────────────────────");
        telemetry.addLine("LB              → Shoot");
        telemetry.addLine("RB / Y          → Hood +/−");
        telemetry.addLine("DPAD U/D        → Speed ± 50");
        telemetry.addLine("R.TRIG          → Intake (free)");
        telemetry.addLine("TRIANGLE        → Reset pose");
        telemetry.addLine("L.TRIG+DPAD     → kF  |  SQ+DPAD → kP");
        telemetry.addLine("A+DPAD          → kD  |  B        → reset");
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

        double currentVel = Math.max(flywheel2.getVelocity(),flywheel1.getVelocity());

        switch (shooterState) {
            case REVING:
                intake.setPower(0);
                if (Math.abs(currentVel - targetVelocity) < 50 || shooterTimer.seconds() > 2) {
                    shooterState = ShooterState.GATE_WAIT;
                    shooterTimer.reset();
                }
                break;

            case GATE_WAIT:
                gate.setPosition(GATE_OPEN);
                intake.setPower(0);
                if (shooterTimer.seconds() > 0.5) {
                    shooterState = ShooterState.SHOOT;
                    shooterTimer.reset();
                }
                break;

            case SHOOT:
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

    // ── Auto-aim turret toward Red Goal ──────────────────────────────────────
    private void autoAim(Pose robotPose) {
        double hRad    = robotPose.getHeading();
        double turretX = robotPose.getX() - TURRET_OFFSET_INCHES * Math.cos(hRad);
        double turretY = robotPose.getY() - TURRET_OFFSET_INCHES * Math.sin(hRad);

        double dx       = Math.abs(RED_GOAL_X - turretX);
        double dy       = Math.abs(RED_GOAL_Y - turretY);
        double distance = Math.sqrt(dx * dx + dy * dy);

        // Auto-set hood + velocity from lookup table
        hoodPosition    = lutInterp(LUT_HOOD, distance);
        targetVelocity  = lutInterp(LUT_VEL,  distance);

        double angleToGoalGlobal = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg   = wrapDeg(Math.toDegrees(hRad));
        double rawDesiredAngle   = -wrapDeg2(angleToGoalGlobal - robotHeadingDeg);

        // Pick whichever equivalent angle (raw, raw±360) falls within [-120, 270]
        // and is closest to the current turret position.
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

    // ── Linear interpolation over a lookup table keyed by LUT_DIST ───────────
    private double lutInterp(double[] table, double dist) {
        if (dist <= LUT_DIST[0])                           return table[0];
        if (dist >= LUT_DIST[LUT_DIST.length - 1])        return table[table.length - 1];
        int i = 0;
        while (i < LUT_DIST.length - 1 && LUT_DIST[i + 1] < dist) i++;
        double t = (dist - LUT_DIST[i]) / (LUT_DIST[i + 1] - LUT_DIST[i]);
        return table[i] + t * (table[i + 1] - table[i]);
    }
}
