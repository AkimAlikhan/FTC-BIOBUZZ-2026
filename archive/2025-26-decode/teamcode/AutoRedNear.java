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
@Autonomous(name = "AUTO_RED_NEAR", group = "Auto")
public class AutoRedNear extends OpMode {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private Follower follower;
    private DcMotorEx intake, flywheel1, flywheel2, bearing;
    private Servo gate, turnAngle1, turnAngle2;
    private static final boolean MIRROR = true;

    // ── Flywheel software PID + feedforward ──────────────────────────────────
    private static final double FLY_KF = 0.00039;
    private static final double FLY_KP = 0.0022;
    private static final double FLY_KI = 0.0;
    private static final double FLY_KD = 0.0;
    private PIDController flyPID;

    // ── Gate ─────────────────────────────────────────────────────────────────
    private static final double GATE_OPEN  = 0.2;
    private static final double GATE_CLOSE = 0.33;

    // ── Lookup table: distance (in) → hood / velocity / time-of-flight ───────
    private static final double[] LUT_DIST = { 40,     50,     60,     70,     80,     90,     100,    110,    120,    130,    140,    150,    160,    170    };
    private static final double[] LUT_VEL  = { 1100,   1150,   1200,   1250,   1300,   1375,   1425,   1475,   1525,   1600,   1625,   1675,   1725,   1825   };
    private static final double[] LUT_HOOD = { 0.35,   0.5,    0.55,   0.65,   0.7,    0.75,   0.8,    0.8,    0.85,   0.85,   0.85,   0.9,    1.0,    1.0    };

    // ── Goal coordinates (Red Alliance) ──────────────────────────────────────
    private static final double GOAL_X = 139.0;
    private static final double GOAL_Y = 139.0;

    // ── Turret constants ──────────────────────────────────────────────────────
    private static final double TICKS_PER_MOTOR_REV = 383.6;
    private static final double TURRET_GEAR_RATIO    = 125.0 / 33.0;
    private static final double TICKS_PER_DEG        = TICKS_PER_MOTOR_REV * TURRET_GEAR_RATIO / 360.0;
    private static final double TURRET_OFFSET_INCHES = 25.83 / 25.4;
    private static final double TURRET_KP            = 0.04;
    private static final double TURRET_KD            = 0.00011;
    private static final double TURRET_TOLERANCE     = 0.5;
    private static final double TURRET_MAX_POWER     =    1.0;
    // Turret limits change after the first shot:
    //   Phase 1 (approach + shoot1): 0 → 270  (positive side only)
    //   Phase 2 (rest of auto):   -120 → 100  (full range)
    private double turretMaxDeg = 270;
    private double turretMinDeg =   0;

    private static final double SHOOT_AIM_TOLERANCE_DEG = 10.0;  // shoot when aimed within this
    private static final double PATH_TIMEOUT            =  2;  // max seconds to wait for isBusy()

    private double lastBearingError = 0;
    private double        turretErrorDeg  = 180;  // start large so we don't shoot immediately

    // ── Shooter state ─────────────────────────────────────────────────────────
    private double  targetFlywheelVel     = 1100;
    private double  targetHoodPos         = 0.0;
    private enum ShootPhase { REVING, GATE_WAIT, SHOOT }
    private ShootPhase shootPhase = ShootPhase.REVING;
    private static final double SHOOT_REV_TOLERANCE = 50.0;  // match teleop
    private static final double SHOOT_REV_TIMEOUT   = 2.0;   // match teleop
    private static final double SHOOT_GATE_WAIT_SEC = 0.03;   // match teleop
    private static final double SHOOT_TIME          = 0.5;

    // ── Auto state machine ────────────────────────────────────────────────────
    private enum AutoState {
        // ① Start → Shoot1 linear
        PATH1_TO_SHOOT1,
        // ② Shoot at Shoot1
        SHOOT_1,
        // ③ Shoot1 → Take2 tangential bezier (control1), intake −1
        PATH2_SHOOT1_TO_TAKE2,
        // ④ Take2 → Shoot2 reversed tangential, intake −0.3
        PATH3_TAKE2_TO_SHOOT2,
        // ⑤ Shoot at Shoot2
        SHOOT_2,

        // ══════════════════ Gate pass 1 ══════════════════
        // ⑥–⑦ Shoot2→Gateopen1→Gateintake, no stop, intake −1
        GATE1_OUT,
        // ⑧ Wait 1 s, intake −1
        GATE1_WAIT,
        // ⑨ Gateintake→Shoot2 direct return, intake −0.3
        GATE1_IN,
        // ⑩ Shoot
        GATE1_SHOOT,

        // ══════════════════ Gate pass 2 ══════════════════
        GATE2_OUT,
        GATE2_WAIT,
        GATE2_IN,
        GATE2_SHOOT,

        // ══════════════════ Gate pass 3 ══════════════════
        GATE3_OUT,
        GATE3_WAIT,
        GATE3_IN,
        GATE3_SHOOT,

        // ⑭ Shoot2 → Take1 tangential, intake −1
        PATH_SHOOT2_TO_TAKE1,
        // ⑮ Take1 → Shoot2 reversed tangential, intake −0.5
        PATH_TAKE1_TO_SHOOT2,
        // ⑯ Shoot
        SHOOT_TAKE1,
        // ⑰ Shoot2 → Take3 tangential bezier (control3), intake −1
        PATH_SHOOT2_TO_TAKE3,
        // ⑱ Take3 → End reversed tangential, intake −0.5
        PATH_TAKE3_TO_END,
        // ⑲ Final shoot
        SHOOT_FINAL,
        IDLE
    }

    private AutoState   currentState = AutoState.PATH1_TO_SHOOT1;
    private ElapsedTime timer        = new ElapsedTime();
    private ElapsedTime autoStartTimer = new ElapsedTime();   // gates turret aiming

    // ── Paths ─────────────────────────────────────────────────────────────────
    private Paths paths;

    public static class Paths {
        public PathChain PathStartToShoot1;
        public PathChain PathShoot1ToTake2;
        public PathChain PathTake2ToShoot2;
        // Gate paths — separate gateopen per pass so positions can be tuned independently
        public PathChain PathGateOut1;          // shoot2→gateopen1→gateintake (pass 1)
        public PathChain PathGateOut2;          // shoot2→gateopen2→gateintake (pass 2)
        public PathChain PathGateOut3;          // shoot2→gateopen3→gateintake (pass 3)
        public PathChain PathGateIn;            // gateintake→shoot2 (direct return)
        // Take1 & Take3
        public PathChain PathShoot2ToTake1;
        public PathChain PathTake1ToShoot2;
        public PathChain PathShoot2ToTake3;
        public PathChain PathTake3ToEnd;

        public Paths(Follower follower) {

            // ── Waypoints ─────────────────────────────────────────────────────
            Pose start      = new Pose(112.81,                 136.08,  Math.toRadians(-90));
            Pose shoot1     = new Pose( 91.1,                  85,     Math.toRadians(-90));
            Pose take2      = new Pose(130.3,                  60.288, Math.toRadians(0));
            Pose control1   = new Pose( 87.35561877667142,     70);
            Pose shoot2     = new Pose( 89.33,                 82.8,   Math.toRadians(-37));
            // Gate positions — tune each gateopen independently per pass
            Pose gateopen1  = new Pose(118,                    66,     Math.toRadians(0));
            Pose gateopen2  = new Pose(118,                    66,     Math.toRadians(0));
            Pose gateopen3  = new Pose(118,                    66,     Math.toRadians(0));
            Pose control2   = new Pose(104.6817861540771,      62);
            Pose gateintake = new Pose(133,                    61,     Math.toRadians(43));
            Pose take3      = new Pose(133.88,                 40.48,  Math.toRadians(0));
            Pose control3   = new Pose( 75,                    40);
            Pose take1      = new Pose(124.793,                83.4,   Math.toRadians(0));
            Pose end        = new Pose( 95.08769100127267,    115.51316442259777);

            // ① Start → Shoot1 (linear, interpolated heading)
            PathStartToShoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(start, shoot1))
                    .setLinearHeadingInterpolation(start.getHeading(), shoot1.getHeading())
                    .build();

            // ③ Shoot1 → Take2 (tangential bezier, control1)
            PathShoot1ToTake2 = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot1, control1, take2))
                    .setTangentHeadingInterpolation()
                    .build();

            // ④ Take2 → Shoot2 (reversed tangential)
            PathTake2ToShoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(take2, shoot2))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // Gate out — shoot2→gateopen(N)→gateintake, separate path per pass
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

            // Gate in — gateintake → shoot2 directly (no intermediate gateclose stop)
            PathGateIn = follower.pathBuilder()
                    .addPath(new BezierLine(gateintake, shoot2))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // ⑭ Shoot2 → Take1 (tangential)
            PathShoot2ToTake1 = follower.pathBuilder()
                    .addPath(new BezierLine(shoot2, take1))
                    .setTangentHeadingInterpolation()
                    .build();

            // ⑮ Take1 → Shoot2 (reversed tangential)
            PathTake1ToShoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(take1, shoot2))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // ⑰ Shoot2 → Take3 (tangential bezier, control3)
            PathShoot2ToTake3 = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot2, control3, take3))
                    .setTangentHeadingInterpolation()
                    .build();

            // ⑱ Take3 → End (reversed tangential)
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
        follower = Constants.createFollower(hardwareMap);

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
        telemetry.addLine("AUTO RED NEAR READY");
        telemetry.update();
    }

    // =========================================================================
    // START
    // =========================================================================
    @Override
    public void start() {
        follower.setStartingPose(new Pose(112.81, 136.08, Math.toRadians(-90)));
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

        // ── Distance to goal ──────────────────────────────────────────────────
        double hRad    = currentPose.getHeading();
        double turretX = currentPose.getX() - TURRET_OFFSET_INCHES * Math.cos(hRad);
        double turretY = currentPose.getY() - TURRET_OFFSET_INCHES * Math.sin(hRad);
        double distToGoal = Math.hypot(GOAL_X - turretX, GOAL_Y - turretY);

        targetHoodPos     = lutInterp(LUT_HOOD, distToGoal);
        targetFlywheelVel = lutInterp(LUT_VEL,  distToGoal);
        setHood(targetHoodPos);

        // ── Turret: always auto-aim ───────────────────────────────────────────
        autoAimPedroStyle(GOAL_X, GOAL_Y, currentPose);
        runCustomShooterControl(targetFlywheelVel);

        // ── State machine ─────────────────────────────────────────────────────
        switch (currentState) {

            // ─────────────────── Opening sequence ─────────────────────────────

            // ① Start → Shoot1 linear
            case PATH1_TO_SHOOT1:
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    currentState = AutoState.SHOOT_1;
                    shootPhase = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;

            // ② Shoot all balls at Shoot1
            case SHOOT_1:
                if (runShooterSequence()) {
                    // Switch turret to full range for the rest of auto
                    turretMinDeg = -120;
                    turretMaxDeg =  100;
                    intake.setPower(-1.0);
                    currentState = AutoState.PATH2_SHOOT1_TO_TAKE2;
                    timer.reset();

                    follower.followPath(paths.PathShoot1ToTake2, false);
                }
                break;

            // ③ Shoot1 → Take2 tangential bezier, intake −1
            case PATH2_SHOOT1_TO_TAKE2:
                intake.setPower(-1.0);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(-0.3);
                    currentState = AutoState.PATH3_TAKE2_TO_SHOOT2;
                    shootPhase = ShootPhase.GATE_WAIT;
                    timer.reset();

                    follower.followPath(paths.PathTake2ToShoot2);
                }
                break;

            // ④ Take2 → Shoot2 reversed tangential, intake −0.3
            case PATH3_TAKE2_TO_SHOOT2:
                intake.setPower(-0.3);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(0);
                    currentState = AutoState.SHOOT_2;
                    shootPhase = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;

            // ⑤ Shoot at Shoot2
            case SHOOT_2:
                if (runShooterSequence()) {
                    intake.setPower(-1.0);
                    currentState = AutoState.GATE1_OUT;
                    timer.reset();
                    follower.setMaxPower(0.8);
                    follower.followPath(paths.PathGateOut1, false);
                }
                break;

            // ═══════════════════ Gate pass 1 ══════════════════════════════════

            // Shoot2→Gateopen1→Gateintake, intake −1
            case GATE1_OUT:
                intake.setPower(-1.0);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    currentState = AutoState.GATE1_WAIT;
                    timer.reset();
                }
                break;

            // Wait 1 s, intake −1
            case GATE1_WAIT:
                intake.setPower(-1.0);
                if (timer.seconds() >= 1) {
                    currentState = AutoState.GATE1_IN;
                    timer.reset();
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.PathGateIn);
                }
                break;

            // Gateintake→Shoot2 direct, intake −0.3
            case GATE1_IN:
                intake.setPower(-0.3);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(0);
                    currentState = AutoState.GATE1_SHOOT;
                    shootPhase = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;

            // Shoot
            case GATE1_SHOOT:
                if (runShooterSequence()) {
                    intake.setPower(-1.0);
                    currentState = AutoState.GATE2_OUT;
                    timer.reset();
                    follower.setMaxPower(0.8);
                    follower.followPath(paths.PathGateOut2);
                }
                break;

            // ═══════════════════ Gate pass 2 ══════════════════════════════════

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
                    shootPhase = ShootPhase.GATE_WAIT;
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

            // ═══════════════════ Gate pass 3 ══════════════════════════════════

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
                    shootPhase = ShootPhase.GATE_WAIT;
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

            // ─────────────────── Take1 ────────────────────────────────────────

            // ⑭ Shoot2→Take1 tangential, intake −1
            case PATH_SHOOT2_TO_TAKE1:
                intake.setPower(-1.0);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    currentState = AutoState.PATH_TAKE1_TO_SHOOT2;
                    shootPhase = ShootPhase.GATE_WAIT;
                    timer.reset();

                    follower.followPath(paths.PathTake1ToShoot2);
                }
                break;

            // ⑮ Take1→Shoot2 reversed tangential, intake −0.5
            case PATH_TAKE1_TO_SHOOT2:
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    currentState = AutoState.SHOOT_TAKE1;
                    shootPhase = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;

            // ⑯ Shoot
            case SHOOT_TAKE1:
                if (runShooterSequence()) {
                    intake.setPower(-1.0);
                    currentState = AutoState.PATH_SHOOT2_TO_TAKE3;
                    timer.reset();

                    follower.followPath(paths.PathShoot2ToTake3);
                }
                break;

            // ─────────────────── Take3 ────────────────────────────────────────

            // ⑰ Shoot2→Take3 tangential bezier (control3), intake −1
            case PATH_SHOOT2_TO_TAKE3:
                intake.setPower(-1.0);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    currentState = AutoState.PATH_TAKE3_TO_END;
                    shootPhase = ShootPhase.GATE_WAIT;
                    timer.reset();

                    follower.followPath(paths.PathTake3ToEnd);
                }
                break;

            // ⑱ Take3→End reversed tangential, intake −0.5
            case PATH_TAKE3_TO_END:
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    currentState = AutoState.SHOOT_FINAL;
                    shootPhase = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;

            // ⑲ Final shoot
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

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addData("Pose",         follower.getPose());

        telemetry.addData("State",         currentState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Dist Goal",     "%.1f in", distToGoal);
        telemetry.addData("Target vel",    "%.0f t/s", targetFlywheelVel);
        telemetry.addData("FW1 Actual",    "%.0f", flywheel1.getVelocity());
        telemetry.addData("FW2 Actual",    "%.0f", flywheel2.getVelocity());
        telemetry.addData("Hood",          "%.2f", targetHoodPos);
        telemetry.addData("Aiming",        autoStartTimer.seconds() >= 1.0 ? "YES" : "waiting 1s");

        telemetry.update();
    }

    // =========================================================================
    // STOP — save pose to blackboard for TeleOp handoff
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

    /**
     * Mirrors the teleop shooter state machine exactly:
     *   REVING    — intake off, wait until |vel − target| < 50 or 2 s timeout
     *   GATE_WAIT — gate open, intake off, hold 0.1 s (lets gate clear)
     *   SHOOT     — intake −1, hold 1 s, then close gate and return true
     */
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
                    shootPhase = ShootPhase.GATE_WAIT;  // reset for next shot
                    return true;
                }
                break;
        }
        return false;
    }

    /** Flywheel power via software PID + feedforward (always running). */
    private void runCustomShooterControl(double targetVel) {
        double avgVel = Math.max(flywheel1.getVelocity(), flywheel2.getVelocity());
        double ff     = FLY_KF * targetVel;
        double pid    = flyPID.calculate(avgVel, targetVel);
        double power  = Math.max(-1.0, Math.min(1.0, ff + pid));
        flywheel1.setPower(power);
        flywheel2.setPower(power);
    }

    /** Set both hood servos (mirrors turnAngle2 when MIRROR = true). */
    private void setHood(double pos) {
        turnAngle1.setPosition(pos);
        turnAngle2.setPosition(MIRROR ? 1.0 - pos : pos);
    }

    /** Linear interpolation over LUT_DIST. */
    private double lutInterp(double[] table, double dist) {
        if (dist <= LUT_DIST[0])                    return table[0];
        if (dist >= LUT_DIST[LUT_DIST.length - 1]) return table[table.length - 1];
        int i = 0;
        while (i < LUT_DIST.length - 1 && LUT_DIST[i + 1] < dist) i++;
        double t = (dist - LUT_DIST[i]) / (LUT_DIST[i + 1] - LUT_DIST[i]);
        return table[i] + t * (table[i + 1] - table[i]);
    }

    /** Returns turret to forward-facing (0°) when not shooting. */

    /** Auto-aim turret toward the goal. */
    private void autoAimPedroStyle(double goalX, double goalY, Pose robotPose) {
        double hRad    = robotPose.getHeading();
        double turretX = robotPose.getX() - TURRET_OFFSET_INCHES * Math.cos(hRad);
        double turretY = robotPose.getY() - TURRET_OFFSET_INCHES * Math.sin(hRad);

        double dx = Math.abs(goalX - turretX);
        double dy = Math.abs(goalY - turretY);

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
        turretErrorDeg     = error;   // used by runShooterSequence to gate firing
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

    /** Returns true when the state machine is in a shooting phase. */
    private boolean isShootingState() {
        switch (currentState) {
            case SHOOT_1: case SHOOT_2:
            case GATE1_SHOOT: case GATE2_SHOOT: case GATE3_SHOOT:
            case SHOOT_TAKE1: case SHOOT_FINAL:
                return true;
            default:
                return false;
        }
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
