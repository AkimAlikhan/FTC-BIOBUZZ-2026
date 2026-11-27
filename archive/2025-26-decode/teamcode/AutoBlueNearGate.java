package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Config
@Autonomous(name = "Blue_4Gate", group = "Auto")
public class AutoBlueNearGate extends OpMode {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private Follower follower;
    private DcMotorEx intake, flywheel1, flywheel2, bearing;
    private Servo gate, turnAngle1, turnAngle2;
    private static final boolean MIRROR = true;

    // ── Flywheel software PID + feedforward ──────────────────────────────────
    private static final double FLY_KF = 0.00039;
    private static final double FLY_KP = 0.003;
    private static final double FLY_KI = 0.0;
    private static final double FLY_KD = 0.0;
    private PIDController flyPID;

    // ── Gate ─────────────────────────────────────────────────────────────────
    private static final double GATE_OPEN  = 0.2;
    private static final double GATE_CLOSE = 0.33;

    // ── Lookup table: distance (in) → hood / velocity ─────────────────────────
    private static final double[] LUT_DIST = { 40,   50,   60,   70,   80,   90,   100,  110,  120,  130,  140,  150,  160,  170  };
    private static final double[] LUT_VEL  = { 1100, 1150, 1200, 1250, 1300, 1375, 1425, 1475, 1525, 1600, 1625, 1675, 1725, 1825 };
    private static final double[] LUT_HOOD = { 0.35, 0.5,  0.55, 0.65, 0.7,  0.75, 0.8,  0.8,  0.85, 0.85, 0.85, 0.9,  1.0,  1.0  };

    // ── Goal coordinates (Blue Alliance) — newX = 144 - redX ─────────────────
    private static final double GOAL_X = 5.0;    // was 139  → 144-139 = 5
    private static final double GOAL_Y = 139.0;  // unchanged

    // ── Turret constants ──────────────────────────────────────────────────────
    private static final double TICKS_PER_MOTOR_REV = 383.6;
    private static final double TURRET_GEAR_RATIO    = 125.0 / 33.0;
    private static final double TICKS_PER_DEG        = TICKS_PER_MOTOR_REV * TURRET_GEAR_RATIO / 360.0;
    private static final double TURRET_OFFSET_INCHES = 25.83 / 25.4;
    private static final double TURRET_KP            = 0.04;
    private static final double TURRET_KD            = 0.00011;
    private static final double TURRET_TOLERANCE     = 0.5;
    private static final double TURRET_MAX_POWER = 1.0;
    // Full range used throughout — no phase switching for blue
    private static final double turretMaxDeg =  270;
    private static final double turretMinDeg = -120;

    private static final double SHOOT_AIM_TOLERANCE_DEG = 10.0;
    private static final double PATH_TIMEOUT            = 2.0;

    private double lastBearingError = 0;
    private double        turretErrorDeg   = 180;

    // ── Shooter state ─────────────────────────────────────────────────────────
    private double     targetFlywheelVel     = 1100;
    private double     targetHoodPos         = 0.0;
    private enum ShootPhase { REVING, GATE_WAIT, SHOOT }
    private ShootPhase shootPhase = ShootPhase.REVING;
    private static final double SHOOT_REV_TOLERANCE = 50.0;
    private static final double SHOOT_REV_TIMEOUT   = 2.0;
    private static final double SHOOT_GATE_WAIT_SEC = 0.03;
    private static final double SHOOT_TIME          = 0.5;

    // ── Auto state machine ────────────────────────────────────────────────────
    private enum AutoState {
        PATH1_TO_SHOOT1, SHOOT_1,
        PATH2_SHOOT1_TO_TAKE2, PATH3_TAKE2_TO_SHOOT2, SHOOT_2,
        GATE1_OUT, GATE1_WAIT, GATE1_IN, GATE1_SHOOT,
        GATE2_OUT, GATE2_WAIT, GATE2_IN, GATE2_SHOOT,
        GATE3_OUT, GATE3_WAIT, GATE3_IN, GATE3_SHOOT,
        PATH_SHOOT2_TO_TAKE1, PATH_TAKE1_TO_SHOOT2, SHOOT_TAKE1,
        PATH_SHOOT2_TO_TAKE3, PATH_TAKE3_TO_END, SHOOT_FINAL, IDLE
    }

    private AutoState   currentState  = AutoState.PATH1_TO_SHOOT1;
    private ElapsedTime timer         = new ElapsedTime();
    private ElapsedTime autoStartTimer = new ElapsedTime();

    // ── Paths ─────────────────────────────────────────────────────────────────
    private Paths paths;

    public static class Paths {
        public PathChain PathStartToShoot1;
        public PathChain PathShoot1ToTake2;
        public PathChain PathTake2ToShoot2;
        public PathChain PathGateOut1;   // shoot2→gateopen1→gateintake (pass 1)
        public PathChain PathGateOut2;   // shoot2→gateopen2→gateintake (pass 2)
        public PathChain PathGateOut3;   // shoot2→gateopen3→gateintake (pass 3)
        public PathChain PathGateIn;     // gateintake→shoot2 direct
        public PathChain PathShoot2ToTake1;
        public PathChain PathTake1ToShoot2;
        public PathChain PathShoot2ToTake3;
        public PathChain PathTake3ToEnd;

        public Paths(Follower follower) {

            // ── Waypoints ─────────────────────────────────────────────────────
            Pose start      = new Pose( 33.155,  136.25,  Math.toRadians(-90));
            Pose shoot1     = new Pose( 52.9,     85,     Math.toRadians(-90));
            Pose take2      = new Pose( 13.7,     61.288, Math.toRadians(180));
            Pose control1   = new Pose( 56.644,   70);
            Pose shoot2     = new Pose( 54.67,    82.8,   Math.toRadians(-143));
            // Gate positions — tune each independently per pass
            Pose gateopen1  = new Pose( 23,       67,     Math.toRadians(180));
            Pose gateopen2  = new Pose( 23,       67,     Math.toRadians(180));
            Pose gateopen3  = new Pose( 23,       67,     Math.toRadians(180));
            Pose control2   = new Pose( 39.318,   62);
            Pose gateintake = new Pose(  9,       61,     Math.toRadians(134));
            Pose take3      = new Pose( 10.12,    42.48,  Math.toRadians(180));
            Pose control3   = new Pose( 69,       39);
            Pose take1      = new Pose( 19.207,   85,     Math.toRadians(180));
            Pose end        = new Pose( 48.912,  115.513);

            PathStartToShoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(start, shoot1))
                    .setLinearHeadingInterpolation(start.getHeading(), shoot1.getHeading())
                    .build();

            PathShoot1ToTake2 = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot1, control1, take2))
                    .setTangentHeadingInterpolation()
                    .build();

            PathTake2ToShoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(take2, shoot2))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            PathGateOut1 = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot2, control2, gateopen1))
                    .setTangentHeadingInterpolation()
                    .addPath(new BezierLine(gateopen1, gateintake))
                    .setLinearHeadingInterpolation(gateopen1.getHeading(), gateintake.getHeading())
                    .build();

            PathGateOut2 = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot2, control2, gateopen2))
                    .setTangentHeadingInterpolation()
                    .addPath(new BezierLine(gateopen2, gateintake))
                    .setLinearHeadingInterpolation(gateopen2.getHeading(), gateintake.getHeading())
                    .build();

            PathGateOut3 = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot2, control2, gateopen3))
                    .setTangentHeadingInterpolation()
                    .addPath(new BezierLine(gateopen3, gateintake))
                    .setLinearHeadingInterpolation(gateopen3.getHeading(), gateintake.getHeading())
                    .build();

            PathGateIn = follower.pathBuilder()
                    .addPath(new BezierLine(gateintake, shoot2))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            PathShoot2ToTake1 = follower.pathBuilder()
                    .addPath(new BezierLine(shoot2, take1))
                    .setTangentHeadingInterpolation()
                    .build();

            PathTake1ToShoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(take1, shoot2))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            PathShoot2ToTake3 = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot2, control3, take3))
                    .setTangentHeadingInterpolation()
                    .build();

            PathTake3ToEnd = follower.pathBuilder()
                    .addPath(new BezierLine(take3, end))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }
    }

    // =========================================================================
    // INIT
    // =========================================================================
    @Override
    public void init() {
        follower  = Constants.createFollower(hardwareMap);
        intake    = hardwareMap.get(DcMotorEx.class, "intake");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        bearing   = hardwareMap.get(DcMotorEx.class, "bearing");
        gate       = hardwareMap.get(Servo.class, "gate");
        turnAngle1 = hardwareMap.get(Servo.class, "turnAngle1");
        turnAngle2 = hardwareMap.get(Servo.class, "turnAngle2");

        flyPID = new PIDController(FLY_KP, FLY_KI, FLY_KD);
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel1.setPower(0);
        flywheel2.setPower(0);

        bearing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bearing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bearing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bearing.setDirection(DcMotor.Direction.REVERSE);

        gate.setPosition(GATE_CLOSE);
        setHood(targetHoodPos);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("AUTO BLUE NEAR READY");
        telemetry.update();
    }

    // =========================================================================
    // START
    // =========================================================================
    @Override
    public void start() {
        follower.setStartingPose(new Pose(33.155, 136.25, Math.toRadians(-90)));
        paths = new Paths(follower);
        follower.setMaxPower(1.0);
        follower.followPath(paths.PathStartToShoot1, false);
        currentState = AutoState.PATH1_TO_SHOOT1;
        timer.reset();
        autoStartTimer.reset();
    }

    // =========================================================================
    // LOOP
    // =========================================================================
    @Override
    public void loop() {
        follower.update();
        Pose currentPose = follower.getPose();

        double hRad    = currentPose.getHeading();
        double turretX = currentPose.getX() - TURRET_OFFSET_INCHES * Math.cos(hRad);
        double turretY = currentPose.getY() - TURRET_OFFSET_INCHES * Math.sin(hRad);
        double distToGoal = Math.hypot(GOAL_X - turretX, GOAL_Y - turretY);

        targetHoodPos     = lutInterp(LUT_HOOD, distToGoal);
        targetFlywheelVel = lutInterp(LUT_VEL,  distToGoal);
        setHood(targetHoodPos);

        autoAimPedroStyle(GOAL_X, GOAL_Y, currentPose);
        runCustomShooterControl(targetFlywheelVel);

        switch (currentState) {

            case PATH1_TO_SHOOT1:
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    currentState = AutoState.SHOOT_1;
                    shootPhase   = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;

            case SHOOT_1:
                if (runShooterSequence()) {
                    intake.setPower(-1.0);
                    currentState = AutoState.PATH2_SHOOT1_TO_TAKE2;
                    timer.reset();
                    follower.followPath(paths.PathShoot1ToTake2, false);
                }
                break;

            case PATH2_SHOOT1_TO_TAKE2:
                intake.setPower(-1.0);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(-0.3);
                    currentState = AutoState.PATH3_TAKE2_TO_SHOOT2;
                    shootPhase   = ShootPhase.GATE_WAIT;
                    timer.reset();
                    follower.followPath(paths.PathTake2ToShoot2);
                }
                break;

            case PATH3_TAKE2_TO_SHOOT2:
                intake.setPower(-0.3);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(0);
                    currentState = AutoState.SHOOT_2;
                    shootPhase   = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;

            case SHOOT_2:
                if (runShooterSequence()) {
                    intake.setPower(-1.0);
                    currentState = AutoState.GATE1_OUT;
                    timer.reset();
                    follower.setMaxPower(0.8);
                    follower.followPath(paths.PathGateOut1, false);
                }
                break;

            // ═══════════ Gate pass 1 ═══════════════════════════════════════════

            case GATE1_OUT:
                intake.setPower(-1.0);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    currentState = AutoState.GATE1_WAIT;
                    timer.reset();
                }
                break;

            case GATE1_WAIT:
                intake.setPower(-1.0);
                if (timer.seconds() >= 1.0) {
                    currentState = AutoState.GATE1_IN;
                    timer.reset();
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.PathGateIn);
                }
                break;

            case GATE1_IN:
                intake.setPower(-0.3);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(0);
                    currentState = AutoState.GATE1_SHOOT;
                    shootPhase   = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;

            case GATE1_SHOOT:
                if (runShooterSequence()) {
                    intake.setPower(-1.0);
                    currentState = AutoState.GATE2_OUT;
                    timer.reset();
                    follower.setMaxPower(0.8);
                    follower.followPath(paths.PathGateOut2);
                }
                break;

            // ═══════════ Gate pass 2 ═══════════════════════════════════════════

            case GATE2_OUT:
                intake.setPower(-1.0);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    currentState = AutoState.GATE2_WAIT;
                    timer.reset();
                }
                break;

            case GATE2_WAIT:
                intake.setPower(-1.0);
                if (timer.seconds() >= 1.3) {
                    currentState = AutoState.GATE2_IN;
                    timer.reset();
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.PathGateIn);
                }
                break;

            case GATE2_IN:
                intake.setPower(-0.3);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(0);
                    currentState = AutoState.GATE2_SHOOT;
                    shootPhase   = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;

            case GATE2_SHOOT:
                if (runShooterSequence()) {
                    intake.setPower(-1.0);
                    currentState = AutoState.GATE3_OUT;
                    timer.reset();
                    follower.setMaxPower(0.8);
                    follower.followPath(paths.PathGateOut3, false);
                }
                break;

            // ═══════════ Gate pass 3 ═══════════════════════════════════════════

            case GATE3_OUT:
                intake.setPower(-1.0);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    currentState = AutoState.GATE3_WAIT;
                    timer.reset();
                }
                break;

            case GATE3_WAIT:
                intake.setPower(-1.0);
                if (timer.seconds() >= 1.3) {
                    currentState = AutoState.GATE3_IN;
                    timer.reset();
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.PathGateIn);
                }
                break;

            case GATE3_IN:
                intake.setPower(-0.3);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(0);
                    currentState = AutoState.GATE3_SHOOT;
                    shootPhase   = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;

            case GATE3_SHOOT:
                if (runShooterSequence()) {
                    intake.setPower(-1.0);
                    currentState = AutoState.PATH_SHOOT2_TO_TAKE1;
                    timer.reset();
                    follower.followPath(paths.PathShoot2ToTake1);
                }
                break;

            // ─────────── Take1 ────────────────────────────────────────────────

            case PATH_SHOOT2_TO_TAKE1:
                intake.setPower(-1.0);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    currentState = AutoState.PATH_TAKE1_TO_SHOOT2;
                    shootPhase   = ShootPhase.GATE_WAIT;
                    timer.reset();
                    follower.followPath(paths.PathTake1ToShoot2);
                }
                break;

            case PATH_TAKE1_TO_SHOOT2:
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    currentState = AutoState.SHOOT_TAKE1;
                    shootPhase   = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;

            case SHOOT_TAKE1:
                if (runShooterSequence()) {
                    intake.setPower(-1.0);
                    currentState = AutoState.PATH_SHOOT2_TO_TAKE3;
                    timer.reset();
                    follower.followPath(paths.PathShoot2ToTake3);
                }
                break;

            // ─────────── Take3 ────────────────────────────────────────────────

            case PATH_SHOOT2_TO_TAKE3:
                intake.setPower(-1.0);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    currentState = AutoState.PATH_TAKE3_TO_END;
                    shootPhase   = ShootPhase.GATE_WAIT;
                    timer.reset();
                    follower.followPath(paths.PathTake3ToEnd);
                }
                break;

            case PATH_TAKE3_TO_END:
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    currentState = AutoState.SHOOT_FINAL;
                    shootPhase   = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;

            case SHOOT_FINAL:
                if (runShooterSequence()) {
                    currentState = AutoState.IDLE;
                }
                break;

            case IDLE:
                intake.setPower(0);
                gate.setPosition(GATE_CLOSE);
                break;
        }

        telemetry.addData("Pose",          follower.getPose());
        telemetry.addData("State",         currentState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Dist Goal",     "%.1f in", distToGoal);
        telemetry.addData("Target vel",    "%.0f t/s", targetFlywheelVel);
        telemetry.addData("FW1 Actual",    "%.0f", flywheel1.getVelocity());
        telemetry.addData("FW2 Actual",    "%.0f", flywheel2.getVelocity());
        telemetry.addData("Hood",          "%.2f", targetHoodPos);
        telemetry.update();
    }

    // =========================================================================
    // STOP
    // =========================================================================
    @Override
    public void stop() {
        Pose finalPose = follower.getPose();
        if (finalPose != null) {
            blackboard.put("lastX",       finalPose.getX());
            blackboard.put("lastY",       finalPose.getY());
            blackboard.put("lastHeading", finalPose.getHeading());
        }
        telemetry.addLine("Pose saved to Blackboard.");
        telemetry.update();
    }

    // =========================================================================
    // HELPERS
    // =========================================================================

    private boolean runShooterSequence() {
        double currentVel = Math.max(flywheel1.getVelocity(), flywheel2.getVelocity());
        switch (shootPhase) {
            case REVING:
                intake.setPower(0);
                gate.setPosition(GATE_CLOSE);
                if ((Math.abs(currentVel - targetFlywheelVel) < SHOOT_REV_TOLERANCE
                        && Math.abs(turretErrorDeg) < SHOOT_AIM_TOLERANCE_DEG)
                        || timer.seconds() > SHOOT_REV_TIMEOUT) {
                    shootPhase = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;
            case GATE_WAIT:
                gate.setPosition(GATE_OPEN);
                intake.setPower(0);
                if (timer.seconds() > SHOOT_GATE_WAIT_SEC) {
                    shootPhase = ShootPhase.SHOOT;
                    timer.reset();
                }
                break;
            case SHOOT:
                intake.setPower(-1.0);
                if (timer.seconds() > SHOOT_TIME) {
                    gate.setPosition(GATE_CLOSE);
                    intake.setPower(0);
                    shootPhase = ShootPhase.GATE_WAIT;
                    return true;
                }
                break;
        }
        return false;
    }

    private void runCustomShooterControl(double targetVel) {
        double avgVel = Math.max(flywheel1.getVelocity(), flywheel2.getVelocity());
        double ff     = FLY_KF * targetVel;
        double pid    = flyPID.calculate(avgVel, targetVel);
        double power  = Math.max(-1.0, Math.min(1.0, ff + pid));
        flywheel1.setPower(power);
        flywheel2.setPower(power);
    }

    private void setHood(double pos) {
        turnAngle1.setPosition(pos);
        turnAngle2.setPosition(MIRROR ? 1.0 - pos : pos);
    }

    private double lutInterp(double[] table, double dist) {
        if (dist <= LUT_DIST[0])                    return table[0];
        if (dist >= LUT_DIST[LUT_DIST.length - 1]) return table[table.length - 1];
        int i = 0;
        while (i < LUT_DIST.length - 1 && LUT_DIST[i + 1] < dist) i++;
        double t = (dist - LUT_DIST[i]) / (LUT_DIST[i + 1] - LUT_DIST[i]);
        return table[i] + t * (table[i + 1] - table[i]);
    }

    /**
     * Auto-aim turret toward the goal using SIGNED dx/dy.
     * (Unlike Red auto, the Blue goal is to the LEFT of the robot,
     *  so Math.abs() would give the wrong quadrant — we don't use it here.)
     */
    private void autoAimPedroStyle(double goalX, double goalY, Pose robotPose) {
        double hRad    = robotPose.getHeading();
        double turretX = robotPose.getX() - TURRET_OFFSET_INCHES * Math.cos(hRad);
        double turretY = robotPose.getY() - TURRET_OFFSET_INCHES * Math.sin(hRad);

        double dx = goalX - turretX;   // signed — goal is LEFT, so dx < 0
        double dy = goalY - turretY;   // signed

        double angleToGoalDeg  = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = wrapDeg(Math.toDegrees(hRad));
        double rawDesiredAngle = -wrapDeg2(angleToGoalDeg - robotHeadingDeg);

        double currentAngle = bearing.getCurrentPosition() / TICKS_PER_DEG;
        double bestTarget   = rawDesiredAngle;
        double bestDist     = Double.MAX_VALUE;
        for (int n = -1; n <= 1; n++) {
            double candidate = rawDesiredAngle + n * 360.0;
            if (candidate >= turretMinDeg && candidate <= turretMaxDeg) {
                double d = Math.abs(candidate - currentAngle);
                if (d < bestDist) { bestDist = d; bestTarget = candidate; }
            }
        }

        double constrained = Math.max(turretMinDeg, Math.min(turretMaxDeg, bestTarget));
        double error       = constrained - currentAngle;
        turretErrorDeg     = error;
        telemetry.addData("Turret cur",  "%.1f deg", currentAngle);
        telemetry.addData("Turret need", "%.1f deg", constrained);
        telemetry.addData("Turret err",  "%.1f deg", error);

        if (Math.abs(error) < TURRET_TOLERANCE) {
            bearing.setPower(0);
            lastBearingError = error;
            return;
        }

        double power = error * TURRET_KP + (error - lastBearingError) * TURRET_KD;
        power = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, power));
        lastBearingError = error;

        if (currentAngle > turretMaxDeg && power > 0) power = 0;
        else if (currentAngle < turretMinDeg && power < 0) power = 0;

        bearing.setPower(power);
    }

    private double wrapDeg(double deg) {
        double d = deg % 360.0;
        if (d < 0) d += 360.0;
        if (d >= 360.0) d -= 360.0;
        return d;
    }

    private double wrapDeg2(double deg) {
        double d = deg % 360.0;
        if (d < -180) d += 360.0;
        if (d >= 180)  d -= 360.0;
        return d;
    }
}
