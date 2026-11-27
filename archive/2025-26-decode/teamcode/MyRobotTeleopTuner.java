package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.LinkedList;
import java.util.Queue;

@TeleOp(name = "TeleOP: Manual Tuner", group = "Drive")
public class MyRobotTeleopTuner extends OpMode {

    // --- HARDWARE ---
    private DcMotorEx bearing, intake, flywheel1, flywheel2;
    private Servo turnAngle, gate;

    // --- PEDRO FOLLOWER ---
    private Follower follower;

    // --- TUNING CONSTANTS (Fixed PIDF, we just tune targets now) ---
    public static double kP = 0.025;
    public static double kD = 0.00005;
    private static final int FILTER_WINDOW_SIZE = 4;

    // --- SAFETY LIMITS ---
    private final double TURRET_MAX_DEG = 170;
    private final double TURRET_MIN_DEG = -170;

    // --- STATE VARIABLES ---
    private boolean IS_RED_ALLIANCE = true;
    private Queue<Double> filterBuffer = new LinkedList<>();
    private double lastError = 0;

    // --- FIELD CONSTANTS ---
    private final double RED_GOAL_X = 139, RED_GOAL_Y = 139;
    private final double BLUE_GOAL_X = 5, BLUE_GOAL_Y = 139;

    // --- TURRET PHYSICAL CONSTANTS ---
    private final int MOTOR_SIGN = 1;
    private final int TICKS_PER_MOTOR_REV = 295;
    private final double GEAR_RATIO_TURRET = 5;

    private double gateOpen = 1;
    private double gateClose = 0;
    private int bearingZeroOffsetTicks = 0;
    private double ticksPerDegBig = 0.0;

    // --- SHOOTER STATE MACHINE ---
    private enum ShooterState { IDLE, REVING, SHOOT, RESET }
    private ShooterState shooterState = ShooterState.IDLE;
    private ElapsedTime shooterTimer = new ElapsedTime();
    private boolean isShooting = false;

    // --- MANUAL TUNING VARIABLES ---
    // These are the variables you will change with buttons
    private double targetFlywheelRPM = 1300;
    private double targetHoodPos = 0.5;

    // Button Toggles to prevent super fast scrolling
    private boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

    @Override
    public void init() {
        // 1. Initialize Pedro Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));

        // 2. Map Hardware
        bearing     = hardwareMap.get(DcMotorEx.class, "bearing");
        intake      = hardwareMap.get(DcMotorEx.class, "intake");
        flywheel1   = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2   = hardwareMap.get(DcMotorEx.class, "flywheel2");
        turnAngle   = hardwareMap.get(Servo.class, "turnAngle");
        gate        = hardwareMap.get(Servo.class, "gate");

        flywheel1.setDirection(DcMotor.Direction.REVERSE);

        ticksPerDegBig = GEAR_RATIO_TURRET * (TICKS_PER_MOTOR_REV / 360.0);
        bearingZeroOffsetTicks = bearing.getCurrentPosition();
        bearing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bearing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bearing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bearing.setDirection(DcMotor.Direction.REVERSE);
        gate.setPosition(gateClose);

        telemetry.addLine("Ready. DPAD L/R for Alliance.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_left) IS_RED_ALLIANCE = true;
        if (gamepad1.dpad_right) IS_RED_ALLIANCE = false;
        telemetry.addData("ALLIANCE", IS_RED_ALLIANCE ? "RED" : "BLUE");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- 1. UPDATE PEDRO ---
        follower.update();
        Pose currentPose = follower.getPose();

        // --- 2. DRIVE LOGIC ---
        follower.setMaxPowerScaling(1);
        if (IS_RED_ALLIANCE) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.7, false);
        } else {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.7, false);
        }

        // --- 3. CALCULATE DISTANCE ---
        double goalX = IS_RED_ALLIANCE ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = IS_RED_ALLIANCE ? RED_GOAL_Y : BLUE_GOAL_Y;
        double distToGoal = Math.hypot(goalX - currentPose.getX(), goalY - currentPose.getY());

        // --- 4. MANUAL TUNING CONTROLS ---
        handleTuningControls();

        // Set the servo to our tuned position immediately
        turnAngle.setPosition(targetHoodPos);

        // --- 5. SHOOTER LOGIC ---
        if (gamepad1.left_bumper && !isShooting) {
            isShooting = true;
            shooterState = ShooterState.REVING;
            shooterTimer.reset();
        }
        updateShooterStateMachine();

        // --- 6. AUTO AIM ---
        autoAimPedroStyle(goalX, goalY, currentPose);

        // --- 7. TELEMETRY FOR TUNING ---
        telemetry.addLine("=== TUNING DASHBOARD ===");
        telemetry.addData("Distance to Goal", "%.1f inches", distToGoal);
        telemetry.addData("Target RPM (Dpad U/D)", "%.0f", targetFlywheelRPM);
        telemetry.addData("Hood Position (Dpad L/R)", "%.3f", targetHoodPos);
        telemetry.addLine("------------------------");
        telemetry.addData("Actual RPM", "%.0f", flywheel1.getVelocity());
        telemetry.update();
    }

    private void handleTuningControls() {
        // Adjust Flywheel RPM (Steps of 50)
        if (gamepad1.dpad_up && !lastUp) {
            targetFlywheelRPM += 50;
        }
        if (gamepad1.dpad_down && !lastDown) {
            targetFlywheelRPM -= 50;
        }

        // Adjust Hood Servo (Steps of 0.01)
        if (gamepad1.dpad_right && !lastRight) {
            targetHoodPos += 0.05;
        }
        if (gamepad1.dpad_left && !lastLeft) {
            targetHoodPos -= 0.05;
        }

        // Safety Clamps
        if (targetHoodPos > 1.0) targetHoodPos = 1.0;
        if (targetHoodPos < 0.0) targetHoodPos = 0.0;

        // Save button states
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
    }

    private void updateShooterStateMachine() {
        if (!isShooting) {
            // Intake logic when not shooting
            intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            flywheel1.setVelocity(0); // Stop flywheels to save battery while tuning
            flywheel2.setVelocity(0);
            return;
        }

        double currentVel = flywheel1.getVelocity();
        double upToSpeedThreshold = 50.0;

        switch (shooterState) {
            case REVING:
                // Use the TUNED variables here
                flywheel1.setVelocity(targetFlywheelRPM);
                flywheel2.setVelocity(targetFlywheelRPM);
                intake.setPower(0);

                // Shoot if speed is reached OR 1 second passed
                if (Math.abs(currentVel - targetFlywheelRPM) < upToSpeedThreshold || shooterTimer.seconds() > 2) {
                    shooterState = ShooterState.SHOOT;
                    shooterTimer.reset();
                }
                break;
            case SHOOT:
                gate.setPosition(gateOpen);
                intake.setPower(1);
                if (shooterTimer.seconds() > 0.5) { // Shortened shot time for faster tuning
                    shooterState = ShooterState.RESET;
                    shooterTimer.reset();
                }
                break;
            case RESET:
                isShooting = false;
                intake.setPower(0);
                flywheel1.setVelocity(0);
                flywheel2.setVelocity(0);
                gate.setPosition(gateClose);
                shooterState = ShooterState.IDLE;
                break;
        }
    }

    private void autoAimPedroStyle(double goalX, double goalY, Pose robotPose) {
        // Kept exactly as you had it
        double hRad = robotPose.getHeading();
        double turretX = robotPose.getX();
        double turretY = robotPose.getY();

        double dx = Math.abs(goalX - turretX);
        double dy = Math.abs(goalY - turretY);

        double angleToGoalRad = Math.atan2(dy, dx);
        double angleToGoalDeg = Math.toDegrees(angleToGoalRad);

        if(!IS_RED_ALLIANCE){
            angleToGoalDeg=90-angleToGoalDeg;
            angleToGoalDeg+=90;
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