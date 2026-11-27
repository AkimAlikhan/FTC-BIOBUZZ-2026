package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Config
@TeleOp(name = "TeleOP: Pedro + Shoot on Move", group = "Drive")
public class MyRobotTeleopMecanumFieldRelativeDrive extends OpMode {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private DcMotorEx flywheel1, flywheel2, intake, bearing;
    private Servo gate, turnAngle1, turnAngle2;

    // ── Pedro Pathing ─────────────────────────────────────────────────────────
    private Follower follower;

    // ── Alliance ──────────────────────────────────────────────────────────────
    private boolean IS_RED_ALLIANCE = true;
    private static final double RED_GOAL_X  = 139.0;
    private static final double RED_GOAL_Y  = 139.0;
    private static final double BLUE_GOAL_X =   5.0;
    private static final double BLUE_GOAL_Y = 139.0;

    // ── Flywheel software PID + feedforward ──────────────────────────────────
    private static final double FLY_KF = 0.00039;
    private static final double FLY_KP = 0.007;
    private static final double FLY_KI = 0.0;
    private static final double FLY_KD = 0.0;
    private PIDController flyPID;

    // ── Gate ──────────────────────────────────────────────────────────────────
    private static final double GATE_OPEN  = 0.2;
    private static final double GATE_CLOSE = 0.33;

    // ── Hood (dual servo, mirrored) ───────────────────────────────────────────
    // Set MIRROR = false if both servos face the same direction.
    private static final boolean MIRROR = true;

    // ── Shooter state machine ─────────────────────────────────────────────────
    private enum ShooterState { IDLE, GATE_WAIT, SHOOT, RESET }
    private ShooterState shooterState = ShooterState.IDLE;
    private ElapsedTime  shooterTimer = new ElapsedTime();
    private boolean      isShooting   = false;

    // ── Turret auto-aim constants ─────────────────────────────────────────────
    private static final double TICKS_PER_MOTOR_REV = 383.6;
    private static final double TURRET_GEAR_RATIO    = 125.0 / 33.0;
    private static final double TICKS_PER_DEG        = TICKS_PER_MOTOR_REV * TURRET_GEAR_RATIO / 360.0;
    private static final double TURRET_OFFSET_INCHES = 25.83 / 25.4;
    private static final double TURRET_KP            = 0.04;
    private static final double TURRET_KD            = 0.00011;
    private static final double TURRET_TOLERANCE     = 0.5;   // deg
    private static final double TURRET_MAX_DEG       =  270.0;
    private static final double TURRET_MIN_DEG       = -120.0;
    private static final double TURRET_MAX_POWER     =    1.0;

    // ── Lookup table: distance (in) → hood / velocity / time-of-flight ───────
    private static final double[] LUT_DIST = { 40,     50,     60,     70,     80,     90,     100,    110,    120,    130,    140,    150,    160,    170    };
    private static final double[] LUT_VEL  = { 1100,   1150,   1200,   1250,   1300,   1375,   1425,   1475,   1525,   1600,   1625,   1675,   1725,   1825   };
    private static final double[] LUT_HOOD = { 0.35,   0.5,    0.55,   0.65,   0.7,    0.75,   0.8,    0.8,    0.85,   0.85,   0.85,   0.9,    1.0,    1.0    };
    private static final double[] LUT_TOF  = { 0.800, 0.600, 0.530, 0.583, 0.635, 0.585, 0.670, 0.665, 0.670, 0.705, 0.740, 0.735, 0.800, 0.880 };

    // ── Turret state ──────────────────────────────────────────────────────────
    private double lastBearingError = 0;

    // ── Shoot-on-the-move targets (set each loop from LUT) ───────────────────
    private double targetVelocity = 1300.0;
    private double targetHoodPos  = 0.0;
    private double distToRealGoal = 0.0;  // updated every loop, read by shooter state machine


    private boolean lastTriangle = false;

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
        flyPID = new PIDController(FLY_KP, FLY_KI, FLY_KD);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Turret bearing motor — reset encoder so angle starts at 0
        bearing = hardwareMap.get(DcMotorEx.class, "bearing");
        bearing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bearing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bearing.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        gate       = hardwareMap.get(Servo.class, "gate");
        turnAngle1 = hardwareMap.get(Servo.class, "turnAngle1");
        turnAngle2 = hardwareMap.get(Servo.class, "turnAngle2");
        gate.setPosition(GATE_CLOSE);
        setHood(targetHoodPos);

        // Pedro Pathing — restore pose from blackboard if available
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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Ready.  DPAD ◄ = Red   DPAD ► = Blue");
        telemetry.update();
    }

    @Override
    public void stop() { }

    @Override
    public void start() { }

    // ── Alliance selection before match starts ────────────────────────────────
    @Override
    public void init_loop() {
        if (gamepad1.dpad_left)  IS_RED_ALLIANCE = true;
        if (gamepad1.dpad_right) IS_RED_ALLIANCE = false;
        telemetry.addData("ALLIANCE", IS_RED_ALLIANCE ? "RED ◄" : "BLUE ►");
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void loop() {

        // ── Triangle: manual pose reset ───────────────────────────────────────
        boolean triangle = gamepad1.triangle;
        if (triangle && !lastTriangle) {
            if (!IS_RED_ALLIANCE) {
                follower.setPose(new Pose(131.4, 8.22, Math.toRadians(0)));
            } else {
                follower.setPose(new Pose(10.7, 10.07, Math.toRadians(180)));
            }
        }
        lastTriangle = triangle;

        // ── Update Pedro ──────────────────────────────────────────────────────
        follower.update();
        Pose pose = follower.getPose();

        // ── Field-centric drive ───────────────────────────────────────────────
        driveFieldCentric(
                -gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                -gamepad1.right_stick_x * 0.7,
                pose.getHeading()
        );

        // ── Velocity in inches/sec from Pedro ────────────────────────────────
        Vector vel  = follower.getVelocity();
        double velX = vel.getXComponent();
        double velY = vel.getYComponent();

        double realGoalX = IS_RED_ALLIANCE ? RED_GOAL_X : BLUE_GOAL_X;
        double realGoalY = IS_RED_ALLIANCE ? RED_GOAL_Y : BLUE_GOAL_Y;

        double hRad    = pose.getHeading();
        double turretX = pose.getX() - TURRET_OFFSET_INCHES * Math.cos(hRad);
        double turretY = pose.getY() - TURRET_OFFSET_INCHES * Math.sin(hRad);
        distToRealGoal = Math.hypot(realGoalX - turretX, realGoalY - turretY);

        double tof = lutInterp(LUT_TOF, distToRealGoal);
        double virtualGoalX = realGoalX - velX * tof;
        double virtualGoalY = realGoalY - velY * tof;
        double distToVirt = Math.hypot(virtualGoalX - turretX, virtualGoalY - turretY);
        // Hood + velocity from real goal distance (actual flight path length)
        targetHoodPos  = lutInterp(LUT_HOOD, distToVirt);
        targetVelocity = lutInterp(LUT_VEL,  distToVirt);
        setHood(targetHoodPos);

        // ── Shoot trigger (LB) ────────────────────────────────────────────────
        if (gamepad1.left_bumper && !isShooting) {
            isShooting   = true;
            shooterState = ShooterState.GATE_WAIT;
            shooterTimer.reset();
        }

        // ── Flywheel + gate + intake via state machine ────────────────────────
        runShooterControl();
        updateShooterStateMachine();

        // ── Turret: always aim at virtual goal ───────────────────────────────
        autoAim(virtualGoalX, virtualGoalY, pose);

        // ── Telemetry ─────────────────────────────────────────────────────────
        double distToVirtualGoal = Math.hypot(virtualGoalX - turretX, virtualGoalY - turretY);
        double turretAngle       = bearing.getCurrentPosition() / TICKS_PER_DEG;

        telemetry.addData("Alliance",    IS_RED_ALLIANCE ? "RED" : "BLUE");
        telemetry.addData("State",       shooterState);
        telemetry.addData("Target vel",  "%.0f t/s", targetVelocity);
        telemetry.addData("FW1 Actual",  "%.0f  (err: %+.0f)", flywheel1.getVelocity(), targetVelocity - flywheel1.getVelocity());
        telemetry.addData("FW2 Actual",  "%.0f  (err: %+.0f)", flywheel2.getVelocity(), targetVelocity - flywheel2.getVelocity());
        telemetry.addData("Hood",        "%.2f", targetHoodPos);
        telemetry.addData("Dist (real)", "%.1f in", distToRealGoal);
        telemetry.addData("Dist (virt)", "%.1f in", distToVirtualGoal);
        telemetry.addData("Correction",  "X:%.1f  Y:%.1f", realGoalX - virtualGoalX, realGoalY - virtualGoalY);
        telemetry.addData("ToF",         "%.3f s", tof);
        telemetry.addData("Turret Angle","%.1f deg", turretAngle);
        telemetry.addData("Robot Pos",   pose);

        telemetry.update();

        // ── FTC Dashboard field overlay (robot dot + heading arrow) ───────────
        TelemetryPacket fieldPacket = new TelemetryPacket();
        Canvas c = fieldPacket.fieldOverlay();
        double rx = pose.getX() - 72;
        double ry = pose.getY() - 72;
        c.setFill("blue").setStroke("blue");
        c.fillCircle(rx, ry, 4);
        double hdg = pose.getHeading();
        c.setStroke("white").setStrokeWidth(2);
        c.strokeLine(rx, ry, rx + 8 * Math.cos(hdg), ry + 8 * Math.sin(hdg));
        FtcDashboard.getInstance().sendTelemetryPacket(fieldPacket);
    }

    // ── Flywheel power via software PID + feedforward (always running) ───────
    private void runShooterControl() {
        double avgVel = Math.max(flywheel1.getVelocity(), flywheel2.getVelocity());
        double ff     = FLY_KF * targetVelocity;
        double pid    = flyPID.calculate(avgVel, targetVelocity);
        double pwr    = Math.max(-1.0, Math.min(1.0, ff + pid));
        flywheel1.setPower(pwr);
        flywheel2.setPower(pwr);
    }

    // ── Shooter state machine ─────────────────────────────────────────────────
    private void updateShooterStateMachine() {
        if (!isShooting) {
            intake.setPower(-gamepad1.right_trigger);
            return;
        }

        double currentVel = Math.max(flywheel1.getVelocity(), flywheel2.getVelocity());

        switch (shooterState) {
            case GATE_WAIT:
                gate.setPosition(GATE_OPEN);
                intake.setPower(0);
                if (shooterTimer.seconds() > 0.15) {
                    shooterState = ShooterState.SHOOT;
                    shooterTimer.reset();
                }
                break;

            case SHOOT:
                // Reduce intake power for long shots (>120 in) to improve accuracy
                intake.setPower(distToRealGoal > 120 ? -0.85 : -1);
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

    // ── Auto-aim turret toward goal (virtual for shoot-on-the-move) ──────────
    private void autoAim(double goalX, double goalY, Pose robotPose) {
        double hRad    = robotPose.getHeading();
        double turretX = robotPose.getX() - TURRET_OFFSET_INCHES * Math.cos(hRad);
        double turretY = robotPose.getY() - TURRET_OFFSET_INCHES * Math.sin(hRad);

        double dx = Math.abs(goalX - turretX);
        double dy = Math.abs(goalY - turretY);

        double angleToGoalDeg = Math.toDegrees(Math.atan2(dy, dx));
        if (!IS_RED_ALLIANCE) {
            angleToGoalDeg = 90 - angleToGoalDeg + 90;
        }

        double robotHeadingDeg = wrapDeg(Math.toDegrees(hRad));
        double rawDesiredAngle = -wrapDeg2(angleToGoalDeg - robotHeadingDeg);

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

        double constrained = Math.max(TURRET_MIN_DEG, Math.min(TURRET_MAX_DEG, bestTarget));
        double error       = constrained - currentAngle;
        telemetry.addData("Turret cur",  currentAngle);
        telemetry.addData("Turret need", constrained);

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

    // ── Set both hood servos together ─────────────────────────────────────────
    private void setHood(double pos) {
        turnAngle1.setPosition(pos);
        turnAngle2.setPosition(MIRROR ? 1.0 - pos : pos);
    }

    // ── Field-centric drive (flipped for blue alliance) ───────────────────────
    private void driveFieldCentric(double strafe, double forward, double turn, double headingRad) {
        if (!IS_RED_ALLIANCE) {
            strafe  *= -1;
            forward *= -1;
        }
        double rotX = strafe  * Math.cos(-headingRad) - (-forward) * Math.sin(-headingRad);
        double rotY = strafe  * Math.sin(-headingRad) + (-forward) * Math.cos(-headingRad);
        double fl   = rotY + rotX + turn;
        double bl   = rotY - rotX + turn;
        double fr   = rotY - rotX - turn;
        double br   = rotY + rotX - turn;
        double max  = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        leftFront.setPower(-fl  / max);
        leftRear.setPower(-bl   / max);
        rightFront.setPower(-fr / max);
        rightRear.setPower(-br  / max);
    }

    private double wrapDeg(double deg) {
        double d = deg % 360.0;
        if (d < 0) d += 360.0;
        return d;
    }

    private double wrapDeg2(double deg) {
        double d = deg % 360.0;
        if (d < -180) d += 360.0;
        if (d >= 180)  d -= 360.0;
        return d;
    }

    // ── Linear interpolation over LUT_DIST ────────────────────────────────────
    private double lutInterp(double[] table, double dist) {
        if (dist <= LUT_DIST[0])                    return table[0];
        if (dist >= LUT_DIST[LUT_DIST.length - 1]) return table[table.length - 1];
        int i = 0;
        while (i < LUT_DIST.length - 1 && LUT_DIST[i + 1] < dist) i++;
        double t = (dist - LUT_DIST[i]) / (LUT_DIST[i + 1] - LUT_DIST[i]);
        return table[i] + t * (table[i + 1] - table[i]);
    }
}
