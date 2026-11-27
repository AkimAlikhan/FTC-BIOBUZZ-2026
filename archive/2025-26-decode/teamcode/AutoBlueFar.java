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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Config
@Autonomous(name = "AUTO_BLUE_FAR", group = "Auto")
public class AutoBlueFar extends OpMode {

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
    private static final double GATE_OPEN  = 0.54;
    private static final double GATE_CLOSE = 0.62;

    // ── Lookup table: distance (in) → hood / velocity ────────────────────────
    private static final double[] LUT_DIST = { 40,   50,   60,   70,   80,   90,   100,  110,  120,  130,  140,  150,  160,  170  };
    private static final double[] LUT_VEL  = { 1100, 1150, 1200, 1250, 1300, 1375, 1425, 1475, 1525, 1600, 1625, 1675, 1725, 1825 };
    private static final double[] LUT_HOOD = { 0.25, 0.30, 0.30, 0.35, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.80, 0.80 };

    // ── Goal coordinates (Blue Alliance = mirror of red: 144 - 139) ──────────
    private static final double GOAL_X = 5.0;
    private static final double GOAL_Y = 139.0;

    // ── Turret constants ──────────────────────────────────────────────────────
    private static final double TICKS_PER_MOTOR_REV = 383.6;
    private static final double TURRET_GEAR_RATIO    = 125.0 / 33.0;
    private static final double TICKS_PER_DEG        = TICKS_PER_MOTOR_REV * TURRET_GEAR_RATIO / 360.0;
    private static final double TURRET_OFFSET_INCHES = 25.83 / 25.4;
    private static final double TURRET_KP            = 0.04;
    private static final double TURRET_KD            = 0.00011;
    private static final double TURRET_TOLERANCE     = 0.5;
    private static final double TURRET_MAX_POWER     = 1.0;
    private double turretMaxDeg = 100;
    private double turretMinDeg = -120;

    private static final double SHOOT_AIM_TOLERANCE_DEG = 10.0;
    private static final double PATH_TIMEOUT            = 2;

    private double lastBearingError = 0;
    private double turretErrorDeg   = 180;

    // ── Shooter state ─────────────────────────────────────────────────────────
    private double targetFlywheelVel = 1100;
    private double targetHoodPos      = 0.0;
    private enum ShootPhase { REVING, GATE_WAIT, SHOOT }
    private ShootPhase shootPhase = ShootPhase.REVING;
    private static final double SHOOT_REV_TOLERANCE = 50.0;
    private static final double SHOOT_REV_TIMEOUT   = 2.0;
    private static final double SHOOT_GATE_WAIT_SEC = 0.03;
    private static final double SHOOT_TIME          = 0.5;

    // ── Auto state machine ────────────────────────────────────────────────────
    private enum AutoState {
        PATH_START_TO_SHOOT,      // ① start → shoot
        SHOOT_START,              // ② shoot
        PATH_SHOOT_TO_INTAKE3,    // ③ shoot → intake3 (intake on)
        PATH_INTAKE3_TO_SHOOT,    // ④ intake3 → shoot
        SHOOT_AFTER_INTAKE3,      // ⑤ shoot
        PATH_SHOOT_TO_INTAKE1_A,  // ⑥ shoot → intake1 (intake on)
        PATH_INTAKE1_A_TO_SHOOT,  // ⑦ intake1 → shoot
        SHOOT_AFTER_INTAKE1_A,    // ⑧ shoot
        PATH_SHOOT_TO_INTAKE2,    // ⑨ shoot → intake2 (intake on)
        PATH_INTAKE2_TO_SHOOT,    // ⑩ intake2 → shoot
        SHOOT_AFTER_INTAKE2,      // ⑪ shoot
        PATH_SHOOT_TO_INTAKE1_B,  // ⑫ shoot → intake1 (intake on)
        PATH_INTAKE1_B_TO_SHOOT,  // ⑬ intake1 → shoot
        SHOOT_AFTER_INTAKE1_B,    // ⑭ shoot
        PATH_SHOOT_TO_INTAKE4,    // ⑮ shoot → intake4 (intake on)
        PATH_INTAKE4_TO_SHOOT,    // ⑯ intake4 → shoot
        SHOOT_AFTER_INTAKE4,      // ⑰ shoot
        PATH_SHOOT_TO_END,        // ⑱ shoot → end
        IDLE                      // ⑲ stop
    }

    private AutoState   currentState   = AutoState.PATH_START_TO_SHOOT;
    private ElapsedTime timer          = new ElapsedTime();
    private ElapsedTime autoStartTimer = new ElapsedTime();

    // ── Paths ─────────────────────────────────────────────────────────────────
    private Paths paths;

    public static class Paths {
        public PathChain PathStartToShoot;
        public PathChain PathShootToIntake3;
        public PathChain PathIntake3ToShoot;
        public PathChain PathShootToIntake1A;
        public PathChain PathIntake1AToShoot;
        public PathChain PathShootToIntake2;
        public PathChain PathIntake2ToShoot;
        public PathChain PathShootToIntake1B;
        public PathChain PathIntake1BToShoot;
        public PathChain PathShootToIntake4;
        public PathChain PathIntake4ToShoot;
        public PathChain PathShootToEnd;

        public Paths(Follower follower) {

            // ── Waypoints — mirror of Red Far: x' = 144 - x, y kept, heading 180-θ
            Pose start   = new Pose( 54.957,  8.212, Math.toRadians(90));   // GIVEN (mirrored)
            Pose shoot0  = new Pose( 55.594, 13.658, Math.toRadians(90));   // first shoot
            Pose intake3 = new Pose(  9.0,    9.164, Math.toRadians(180));  // GIVEN (mirrored)
            Pose shoot1  = new Pose( 56.352, 11.941);                       // return shoot
            Pose intake1 = new Pose( 12.0,   37.136, Math.toRadians(180));  // GIVEN (mirrored, used twice)
            Pose shoot2  = new Pose( 55.865, 11.551);                       // return shoot
            Pose intake2 = new Pose( 10.632, 39.426, Math.toRadians(180));  // from path (mirrored)
            Pose shoot3  = new Pose( 55.702, 11.778);                       // return shoot
            Pose shoot4  = new Pose( 56.187, 11.601);                       // return shoot
            Pose intake4 = new Pose( 12.023, 37.680, Math.toRadians(180));  // from path (mirrored)
            Pose shoot5  = new Pose( 55.942, 11.616);                       // return shoot
            Pose end     = new Pose( 37.401, 11.940);                       // from path (mirrored)

            // ── Control points (from path, mirrored) ──────────────────────────
            Pose ctrlIntake3  = new Pose( 52.825, 37.490);
            Pose ctrlIntake1A = new Pose( 37.511,  8.686);
            Pose ctrlIntake2  = new Pose( 16.945,  6.052);
            Pose ctrlIntake1B = new Pose( 32.824,  8.895);
            Pose ctrlIntake4  = new Pose( 25.570,  1.896);

            // ① start → shoot (linear, interpolated heading)
            PathStartToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(start, shoot0))
                    .setLinearHeadingInterpolation(start.getHeading(), shoot0.getHeading())
                    .build();

            // ③ shoot → intake3 (curve, control)
            PathShootToIntake3 = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot0, ctrlIntake3, intake3))
                    .setTangentHeadingInterpolation()
                    .build();

            // ④ intake3 → shoot (reversed)
            PathIntake3ToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(intake3, shoot1))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // ⑥ shoot → intake1 (curve, control)
            PathShootToIntake1A = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot1, ctrlIntake1A, intake1))
                    .setTangentHeadingInterpolation()
                    .build();

            // ⑦ intake1 → shoot (reversed)
            PathIntake1AToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(intake1, shoot2))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // ⑨ shoot → intake2 (curve, control)
            PathShootToIntake2 = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot2, ctrlIntake2, intake2))
                    .setTangentHeadingInterpolation()
                    .build();

            // ⑩ intake2 → shoot (reversed)
            PathIntake2ToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(intake2, shoot3))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // ⑫ shoot → intake1 (curve, control)
            PathShootToIntake1B = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot3, ctrlIntake1B, intake1))
                    .setTangentHeadingInterpolation()
                    .build();

            // ⑬ intake1 → shoot (reversed)
            PathIntake1BToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(intake1, shoot4))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // ⑮ shoot → intake4 (curve, control)
            PathShootToIntake4 = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot4, ctrlIntake4, intake4))
                    .setTangentHeadingInterpolation()
                    .build();

            // ⑯ intake4 → shoot (reversed)
            PathIntake4ToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(intake4, shoot5))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // ⑱ shoot → end
            PathShootToEnd = follower.pathBuilder()
                    .addPath(new BezierLine(shoot5, end))
                    .setTangentHeadingInterpolation()
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
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        bearing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bearing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bearing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bearing.setDirection(DcMotor.Direction.REVERSE);

        gate.setPosition(GATE_CLOSE);
        setHood(targetHoodPos);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("AUTO BLUE FAR READY");
        telemetry.update();
    }

    // =========================================================================
    // START
    // =========================================================================
    @Override
    public void start() {
        follower.setStartingPose(new Pose(54.957, 8.212, Math.toRadians(90)));
        paths = new Paths(follower);

        follower.setMaxPower(1.0);
        follower.followPath(paths.PathStartToShoot, false);
        currentState = AutoState.PATH_START_TO_SHOOT;
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

        // ── Turret + flywheel: always running ─────────────────────────────────
        autoAimPedroStyle(GOAL_X, GOAL_Y, currentPose);
        runCustomShooterControl(targetFlywheelVel);

        // ── State machine ─────────────────────────────────────────────────────
        switch (currentState) {

            // ① start → shoot
            case PATH_START_TO_SHOOT:
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    currentState = AutoState.SHOOT_START;
                    // First shot: auto just started, flywheel isn't up to speed yet,
                    // so gate on rev-up (REVING) instead of firing immediately.
                    shootPhase   = ShootPhase.REVING;
                    timer.reset();
                }
                break;

            // ② shoot
            case SHOOT_START:
                if (runShooterSequence()) {
                    intake.setPower(-1.0);
                    currentState = AutoState.PATH_SHOOT_TO_INTAKE3;
                    timer.reset();
                    follower.followPath(paths.PathShootToIntake3, false);
                }
                break;

            // ③ shoot → intake3
            case PATH_SHOOT_TO_INTAKE3:
                intake.setPower(-1.0);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(-0.3);
                    currentState = AutoState.PATH_INTAKE3_TO_SHOOT;
                    timer.reset();
                    follower.followPath(paths.PathIntake3ToShoot);
                }
                break;

            // ④ intake3 → shoot
            case PATH_INTAKE3_TO_SHOOT:
                intake.setPower(-0.3);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(0);
                    currentState = AutoState.SHOOT_AFTER_INTAKE3;
                    shootPhase   = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;

            // ⑤ shoot
            case SHOOT_AFTER_INTAKE3:
                if (runShooterSequence()) {
                    intake.setPower(-1.0);
                    currentState = AutoState.PATH_SHOOT_TO_INTAKE1_A;
                    timer.reset();
                    follower.followPath(paths.PathShootToIntake1A);
                }
                break;

            // ⑥ shoot → intake1
            case PATH_SHOOT_TO_INTAKE1_A:
                intake.setPower(-1.0);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(-0.3);
                    currentState = AutoState.PATH_INTAKE1_A_TO_SHOOT;
                    timer.reset();
                    follower.followPath(paths.PathIntake1AToShoot);
                }
                break;

            // ⑦ intake1 → shoot
            case PATH_INTAKE1_A_TO_SHOOT:
                intake.setPower(-0.3);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(0);
                    currentState = AutoState.SHOOT_AFTER_INTAKE1_A;
                    shootPhase   = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;

            // ⑧ shoot
            case SHOOT_AFTER_INTAKE1_A:
                if (runShooterSequence()) {
                    intake.setPower(-1.0);
                    currentState = AutoState.PATH_SHOOT_TO_INTAKE2;
                    timer.reset();
                    follower.followPath(paths.PathShootToIntake2);
                }
                break;

            // ⑨ shoot → intake2
            case PATH_SHOOT_TO_INTAKE2:
                intake.setPower(-1.0);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(-0.3);
                    currentState = AutoState.PATH_INTAKE2_TO_SHOOT;
                    timer.reset();
                    follower.followPath(paths.PathIntake2ToShoot);
                }
                break;

            // ⑩ intake2 → shoot
            case PATH_INTAKE2_TO_SHOOT:
                intake.setPower(-0.3);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(0);
                    currentState = AutoState.SHOOT_AFTER_INTAKE2;
                    shootPhase   = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;

            // ⑪ shoot
            case SHOOT_AFTER_INTAKE2:
                if (runShooterSequence()) {
                    intake.setPower(-1.0);
                    currentState = AutoState.PATH_SHOOT_TO_INTAKE1_B;
                    timer.reset();
                    follower.followPath(paths.PathShootToIntake1B);
                }
                break;

            // ⑫ shoot → intake1
            case PATH_SHOOT_TO_INTAKE1_B:
                intake.setPower(-1.0);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(-0.3);
                    currentState = AutoState.PATH_INTAKE1_B_TO_SHOOT;
                    timer.reset();
                    follower.followPath(paths.PathIntake1BToShoot);
                }
                break;

            // ⑬ intake1 → shoot
            case PATH_INTAKE1_B_TO_SHOOT:
                intake.setPower(-0.3);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(0);
                    currentState = AutoState.SHOOT_AFTER_INTAKE1_B;
                    shootPhase   = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;

            // ⑭ shoot
            case SHOOT_AFTER_INTAKE1_B:
                if (runShooterSequence()) {
                    intake.setPower(-1.0);
                    currentState = AutoState.PATH_SHOOT_TO_INTAKE4;
                    timer.reset();
                    follower.followPath(paths.PathShootToIntake4);
                }
                break;

            // ⑮ shoot → intake4
            case PATH_SHOOT_TO_INTAKE4:
                intake.setPower(-1.0);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(-0.3);
                    currentState = AutoState.PATH_INTAKE4_TO_SHOOT;
                    timer.reset();
                    follower.followPath(paths.PathIntake4ToShoot);
                }
                break;

            // ⑯ intake4 → shoot
            case PATH_INTAKE4_TO_SHOOT:
                intake.setPower(-0.3);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    intake.setPower(0);
                    currentState = AutoState.SHOOT_AFTER_INTAKE4;
                    shootPhase   = ShootPhase.GATE_WAIT;
                    timer.reset();
                }
                break;

            // ⑰ shoot
            case SHOOT_AFTER_INTAKE4:
                if (runShooterSequence()) {
                    currentState = AutoState.PATH_SHOOT_TO_END;
                    timer.reset();
                    follower.followPath(paths.PathShootToEnd);
                }
                break;

            // ⑱ shoot → end
            case PATH_SHOOT_TO_END:
                intake.setPower(0);
                if (!follower.isBusy() || timer.seconds() > PATH_TIMEOUT) {
                    currentState = AutoState.IDLE;
                }
                break;

            // ⑲ stop
            case IDLE:
                intake.setPower(0);
                gate.setPosition(GATE_CLOSE);
                break;
        }

        // ── Telemetry ─────────────────────────────────────────────────────────
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

    private boolean runShooterSequence() {
        double currentVel = Math.max(Math.abs(flywheel1.getVelocity()), Math.abs(flywheel2.getVelocity()));

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

    /** Flywheel power via software PID + feedforward (always running). */
    private void runCustomShooterControl(double targetVel) {
        double avgVel = Math.max(Math.abs(flywheel1.getVelocity()), Math.abs(flywheel2.getVelocity()));
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
        if (dist <= LUT_DIST[0])                   return table[0];
        if (dist >= LUT_DIST[LUT_DIST.length - 1]) return table[table.length - 1];
        int i = 0;
        while (i < LUT_DIST.length - 1 && LUT_DIST[i + 1] < dist) i++;
        double t = (dist - LUT_DIST[i]) / (LUT_DIST[i + 1] - LUT_DIST[i]);
        return table[i] + t * (table[i + 1] - table[i]);
    }

    /** Auto-aim turret toward the goal (Blue Alliance — mirrored angle). */
    private void autoAimPedroStyle(double goalX, double goalY, Pose robotPose) {
        double hRad    = robotPose.getHeading();
        double turretX = robotPose.getX() - TURRET_OFFSET_INCHES * Math.cos(hRad);
        double turretY = robotPose.getY() - TURRET_OFFSET_INCHES * Math.sin(hRad);

        double dx = Math.abs(goalX - turretX);
        double dy = Math.abs(goalY - turretY);

        double angleToGoalDeg = Math.toDegrees(Math.atan2(dy, dx));
        // Blue mirror: goal sits on the left side, reflect the angle (same as teleop).
        angleToGoalDeg = 180 - angleToGoalDeg;

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