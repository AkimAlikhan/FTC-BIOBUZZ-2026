package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.LinkedList;
import java.util.Queue;

@Autonomous(name = "Auto: Pedro + Auto Aim + Dynamic Shoot", group = "Auto")
public class PedroAuto extends OpMode {

    // --- HARDWARE ---
    private Follower follower;
    private DcMotorEx intake, flywheel1, flywheel2, bearing;
    private Servo gate, turnAngle;

    // --- PATHS CLASS ---
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, line10;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(new BezierLine(new Pose(32.751, 136.186), new Pose(60.270, 88.191))).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180)).build();
            Path2 = follower.pathBuilder().addPath(new BezierCurve(new Pose(60.270, 88.191), new Pose(46.260, 66.425), new Pose(8.606, 58.596))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
            Path3 = follower.pathBuilder().addPath(new BezierCurve(new Pose(8.606, 58.596), new Pose(70.400, 64.993), new Pose(60.020, 88.488))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
            Path4 = follower.pathBuilder().addPath(new BezierCurve(new Pose(60.020, 88.488), new Pose(61.678, 65.167), new Pose(13.936, 69.382))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
            Path5 = follower.pathBuilder().addPath(new BezierLine(new Pose(13.936, 69.382), new Pose(9.372, 56.056))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(100)).build();
            Path6 = follower.pathBuilder().addPath(new BezierCurve(new Pose(9.372, 56.056), new Pose(43.955, 63.400), new Pose(60.033, 88.671))).setLinearHeadingInterpolation(Math.toRadians(100), Math.toRadians(180)).build();
            Path7 = follower.pathBuilder().addPath(new BezierLine(new Pose(60.033, 88.671), new Pose(13.547, 83.053))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
            Path8 = follower.pathBuilder().addPath(new BezierLine(new Pose(13.547, 83.053), new Pose(60.425, 88.633))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
            Path9 = follower.pathBuilder().addPath(new BezierCurve(new Pose(60.425, 88.633), new Pose(60.282, 34.552), new Pose(15.572, 34.662))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
            line10 = follower.pathBuilder().addPath(new BezierLine(new Pose(15.572, 34.662), new Pose(69.984, 100.349))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
        }
    }

    private Paths paths;

    // --- LOOKUP TABLE DATA (Copied from TeleOp) ---
    private static final double[] DISTANCES =   { 40,   50,   60,   70,   80,   90,    100,  110,  120,  130,  140,  150 };
    private static final double[] VELOCITIES =  { 1100, 1100, 1100, 1150, 1200, 1250,  1300, 1350, 1400, 1550, 1600, 1700 };
    private static final double[] HOOD_ANGLES = { 0.0,  0.0,  0.0,  0.2,  0.2,  0.25,  0.35, 0.4,  0.45, 0.5,  0.5,  0.5 };

    // --- SHOOTER VARIABLES ---
    private double targetFlywheelVel = 1300;
    private double targetHoodPos = 0;
    private double gateOpen = 1;
    private double gateClose = 0;

    // --- TURRET VARIABLES ---
    public static double kP = 0.025;
    public static double kD = 0.00005;
    private static final int FILTER_WINDOW_SIZE = 4;
    private final double TURRET_MAX_DEG = 150;
    private final double TURRET_MIN_DEG = -150;
    private Queue<Double> filterBuffer = new LinkedList<>();
    private double lastError = 0;
    private final int MOTOR_SIGN = 1;
    private final int TICKS_PER_MOTOR_REV = 295;
    private final double GEAR_RATIO_TURRET = 5;
    private int bearingZeroOffsetTicks = 0;
    private double ticksPerDegBig = 0.0;

    // --- FIELD CONSTANTS ---
    private boolean IS_RED_ALLIANCE = false; // Set logic to flip this if needed
    private final double RED_GOAL_X = 139, RED_GOAL_Y = 139;
    private final double BLUE_GOAL_X = 5, BLUE_GOAL_Y = 139;

    // --- STATE MACHINE ---
    private enum AutoState {
        START, TRAJECTORY_1, SHOOT_PRELOAD, TRAJECTORY_2, TRAJECTORY_3, SHOOT_SAMPLE_1,
        TRAJECTORY_4, TRAJECTORY_5, INTAKE_SAMPLE_2, TRAJECTORY_6, SHOOT_SAMPLE_2,
        TRAJECTORY_4_AGAIN, TRAJECTORY_5_AGAIN, INTAKE_SAMPLE_3, TRAJECTORY_6_AGAIN, SHOOT_SAMPLE_3,
        TRAJECTORY_7, TRAJECTORY_8, SHOOT_SAMPLE_4, TRAJECTORY_9, TRAJECTORY_10, SHOOT_SAMPLE_5, IDLE
    }

    private AutoState currentState = AutoState.START;
    private ElapsedTime timer = new ElapsedTime();
    private double SHOOT_TIME = 3;
    private double INTAKE_WAIT_TIME = 3;

    @Override
    public void init() {
        // 1. Init Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(32.751, 136.186, Math.toRadians(90)));
        paths = new Paths(follower);

        // 2. Init Hardware
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        bearing = hardwareMap.get(DcMotorEx.class, "bearing");
        gate = hardwareMap.get(Servo.class, "gate");
        turnAngle = hardwareMap.get(Servo.class, "turnAngle");

        // 3. Configure Motors (Matches TeleOp)
        PIDFCoefficients flyPIDF = new PIDFCoefficients(40, 0, 0, 15);
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flyPIDF);
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flyPIDF);
        flywheel1.setDirection(DcMotor.Direction.REVERSE);

        ticksPerDegBig = GEAR_RATIO_TURRET * (TICKS_PER_MOTOR_REV / 360.0);
        bearingZeroOffsetTicks = bearing.getCurrentPosition();
        bearing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bearing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bearing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bearing.setDirection(DcMotor.Direction.REVERSE);

        gate.setPosition(gateClose);

        telemetry.addLine("Auto Initialized with Tracking. Ready.");
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(AutoState.TRAJECTORY_1);
    }

    @Override
    public void loop() {
        follower.update();
        Pose currentPose = follower.getPose();

        // --- 1. CONTINUOUS TRACKING LOGIC ---
        // Calculate Distance
        double goalX = IS_RED_ALLIANCE ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = IS_RED_ALLIANCE ? RED_GOAL_Y : BLUE_GOAL_Y;
        double distanceToGoal = Math.hypot(goalX - currentPose.getX(), goalY - currentPose.getY());

        // Update Lookup Table Targets

        flywheel1.setVelocity(targetFlywheelVel);
        flywheel2.setVelocity(targetFlywheelVel);
        // Update Servo Hood
        turnAngle.setPosition(0);

        // Aim Turret (Auto Tracking)
        autoAimPedroStyle(goalX, goalY, currentPose);

        // --- 2. STATE MACHINE ---
        switch (currentState) {
            case START: break;

            case TRAJECTORY_1:
                follower.followPath(paths.Path1);
                setPathState(AutoState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (follower.isBusy()) {
                    timer.reset(); // Wait for arrival!
                } else {
                    runShooterSequence(AutoState.TRAJECTORY_2);
                }
                break;

            case TRAJECTORY_2:
                if (!follower.isBusy()) {
                    intake.setPower(1.0);
                    follower.followPath(paths.Path2);
                    setPathState(AutoState.TRAJECTORY_3);
                }
                break;

            case TRAJECTORY_3:
                if (!follower.isBusy()) {
                    intake.setPower(0.3);
                    follower.followPath(paths.Path3);
                    setPathState(AutoState.SHOOT_SAMPLE_1);
                }
                break;

            case SHOOT_SAMPLE_1:
                if (follower.isBusy()) {
                    timer.reset();
                } else {
                    runShooterSequence(AutoState.TRAJECTORY_4);
                }
                break;

            case TRAJECTORY_4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    setPathState(AutoState.TRAJECTORY_5);
                }
                break;

            case TRAJECTORY_5:
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(paths.Path5);
                    setPathState(AutoState.INTAKE_SAMPLE_2);
                }
                break;

            case INTAKE_SAMPLE_2:
                if (!follower.isBusy()) {
                    intake.setPower(1.0);
                    if (timer.seconds() > INTAKE_WAIT_TIME) {
                        intake.setPower(0.3);
                        setPathState(AutoState.TRAJECTORY_6);
                    }
                }
                break;

            case TRAJECTORY_6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6);
                    setPathState(AutoState.SHOOT_SAMPLE_2);
                }
                break;

            case SHOOT_SAMPLE_2:
                if (follower.isBusy()) {
                    timer.reset();
                } else {
                    runShooterSequence(AutoState.TRAJECTORY_4_AGAIN);
                }
                break;

            case TRAJECTORY_4_AGAIN:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    setPathState(AutoState.TRAJECTORY_5_AGAIN);
                }
                break;

            case TRAJECTORY_5_AGAIN:
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(paths.Path5);
                    setPathState(AutoState.INTAKE_SAMPLE_3);
                }
                break;

            case INTAKE_SAMPLE_3:
                if (!follower.isBusy()) {
                    intake.setPower(1.0);
                    if (timer.seconds() > INTAKE_WAIT_TIME) {
                        intake.setPower(0.3);
                        setPathState(AutoState.TRAJECTORY_6_AGAIN);
                    }
                }
                break;

            case TRAJECTORY_6_AGAIN:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6);
                    setPathState(AutoState.SHOOT_SAMPLE_3);
                }
                break;

            case SHOOT_SAMPLE_3:
                if (follower.isBusy()) {
                    timer.reset();
                } else {
                    runShooterSequence(AutoState.TRAJECTORY_7);
                }
                break;

            case TRAJECTORY_7:
                if (!follower.isBusy()) {
                    intake.setPower(1.0);
                    follower.followPath(paths.Path7);
                    setPathState(AutoState.TRAJECTORY_8);
                }
                break;

            case TRAJECTORY_8:
                if (!follower.isBusy()) {
                    intake.setPower(0.3);
                    follower.followPath(paths.Path8);
                    setPathState(AutoState.SHOOT_SAMPLE_4);
                }
                break;

            case SHOOT_SAMPLE_4:
                if (follower.isBusy()) {
                    timer.reset();

                } else {
                    runShooterSequence(AutoState.TRAJECTORY_9);
                }
                break;

            case TRAJECTORY_9:
                if (!follower.isBusy()) {
                    intake.setPower(1.0);
                    follower.followPath(paths.Path9);
                    setPathState(AutoState.TRAJECTORY_10);
                }
                break;

            case TRAJECTORY_10:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(paths.line10);
                    setPathState(AutoState.SHOOT_SAMPLE_5);
                }
                break;

            case SHOOT_SAMPLE_5:
                if (follower.isBusy()) {
                    timer.reset(); // Wait for arrival!
                    flywheel1.setVelocity(targetFlywheelVel);
                    flywheel2.setVelocity(targetFlywheelVel);
                } else {
                    runShooterSequence(AutoState.IDLE);
                }
                break;

            case IDLE:
                intake.setPower(0);
                flywheel1.setVelocity(0);
                flywheel2.setVelocity(0);
                gate.setPosition(gateClose);
                break;
        }

        telemetry.addData("State", currentState);
        telemetry.addData("Dist", "%.1f", distanceToGoal);
        telemetry.addData("Target Vel", "%.0f", targetFlywheelVel);
        telemetry.update();
    }


    private void setPathState(AutoState newState) {
        currentState = newState;
        timer.reset();
    }

    /**
     * Executes the shooting sequence: Rev -> Shoot -> Reset.
     * Automatically transitions to 'nextState' when done.
     */
    private void runShooterSequence(AutoState nextState) {
        // 1. Rev up (using dynamic target)
        // 2. Open Gate
        gate.setPosition(gateOpen);
        intake.setPower(1);
        // 3. Wait for timer
        if (timer.seconds() > SHOOT_TIME) {
            gate.setPosition(gateClose);
            flywheel1.setVelocity(0);
            flywheel2.setVelocity(0);
            intake.setPower(0);
            setPathState(nextState);
        }
    }

//    private void calculateShooterTargets(double dist) {
//        if (dist <= DISTANCES[0]) {
//            targetFlywheelVel = VELOCITIES[0];
//            targetHoodPos = HOOD_ANGLES[0];
//            return;
//        }
//        if (dist >= DISTANCES[DISTANCES.length - 1]) {
//            targetFlywheelVel = VELOCITIES[VELOCITIES.length - 1];
//            targetHoodPos = HOOD_ANGLES[HOOD_ANGLES.length - 1];
//            return;
//        }
//        int index = 0;
//        while (index < DISTANCES.length - 1 && DISTANCES[index + 1] < dist) index++;
//        double dist1 = DISTANCES[index], dist2 = DISTANCES[index + 1];
//        double vel1 = VELOCITIES[index], vel2 = VELOCITIES[index + 1];
//        double hood1 = HOOD_ANGLES[index], hood2 = HOOD_ANGLES[index + 1];
//        double t = (dist - dist1) / (dist2 - dist1);
//        targetFlywheelVel = vel1 + (t * (vel2 - vel1));
//        targetHoodPos = hood1 + (t * (hood2 - hood1));
//    }

    private void autoAimPedroStyle(double goalX, double goalY, Pose robotPose) {
        double hRad = robotPose.getHeading();
        double turretX = robotPose.getX();
        double turretY = robotPose.getY();

        double dx = Math.abs(goalX - turretX);
        double dy = Math.abs(goalY - turretY);
        double angleToGoalDeg = Math.toDegrees(Math.atan2(dy, dx));

        if(!IS_RED_ALLIANCE){
            angleToGoalDeg = 90 - angleToGoalDeg;
            angleToGoalDeg += 90;
        }

        double robotHeadingDeg = wrapDeg(Math.toDegrees(hRad));
        double rawDesiredAngle = wrapDeg2(angleToGoalDeg - robotHeadingDeg);

        filterBuffer.add(rawDesiredAngle);
        if (filterBuffer.size() > FILTER_WINDOW_SIZE) filterBuffer.remove();
        double smoothedTarget = 0;
        for (Double val : filterBuffer) smoothedTarget += val;
        smoothedTarget /= filterBuffer.size();

        double constrainedTarget = Math.max(TURRET_MIN_DEG, Math.min(TURRET_MAX_DEG, smoothedTarget));
        double currentAngle = (bearing.getCurrentPosition() - bearingZeroOffsetTicks) / ticksPerDegBig;
        double error = constrainedTarget - currentAngle;

        if (Math.abs(error) < 1.0) {
            bearing.setPower(0);
            lastError = error;
            return;
        }
        double power = (error * kP) + ((error - lastError) * kD);
        power = Math.max(-1.0, Math.min(1.0, power));
        lastError = error;

        if (currentAngle > TURRET_MAX_DEG && power * MOTOR_SIGN > 0) power = 0;
        else if (currentAngle < TURRET_MIN_DEG && power * MOTOR_SIGN < 0) power = 0;

        bearing.setPower(power * MOTOR_SIGN);
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
        if (d >= 180) d -= 360.0;
        return d;
    }
}