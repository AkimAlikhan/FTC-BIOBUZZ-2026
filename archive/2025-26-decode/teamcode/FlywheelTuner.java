package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

@TeleOp(name = "TUNING: Flywheel Velocity", group = "Tuning")
public class FlywheelTuner extends OpMode {

    // --- Hardware ---
    private DcMotorEx bearing, intake, flywheel1, flywheel2;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private Servo turnAngle, flicker;

    private Follower follower;
    private GoBildaPinpointDriver pinpoint;

    // --- Constants ---
    public static double kP = 0.02;
    public static double kD = 0.0001;
    public static double PREDICT_FACTOR = 0;
    private static final int FILTER_WINDOW_SIZE = 4;
    private final double TURRET_MAX_DEG = 80.0;
    private final double TURRET_MIN_DEG = -110.0;
    private final double GEAR_RATIO_TURRET = 2.31;
    private final int TICKS_PER_MOTOR_REV = 112;
    private final int MOTOR_SIGN = 1;

    // --- Tuning Variables ---
    private double targetVelocity = 1500; // Start at a reasonable guess
    private boolean isFlywheelRunning = false;
    private ArrayList<String> dataPoints = new ArrayList<>();

    // --- State Variables ---
    private final double BLUE_GOAL_X =5, BLUE_GOAL_Y = 134; // Tuning for Blue Goal
    private Queue<Double> filterBuffer = new LinkedList<>();
    private double lastError = 0;
    private int bearingZeroOffsetTicks = 0;
    private double ticksPerDegBig = 0.0;
    private double telemetryBearingCurrent = 0;
    private double telemetryBearingTarget = 0;

    // Button toggle states
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastX = false;
    private boolean lastSquare = false;

    @Override
    public void init() {
        // 1. Init Follower
        follower = Constants.createFollower(hardwareMap);

        // 2. Init Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // 3. Init Motors
        leftFront = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        leftRear = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        rightFront = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        rightRear = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        // 4. Set Fixed Start Pose (9, 9, 90 degrees)
        Pose startPose = new Pose(9, 9, Math.toRadians(90));
        follower.setStartingPose(startPose);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 9, 9, AngleUnit.DEGREES, 90));
        pinpoint.update();

        // 5. Init Mechanisms
        bearing = hardwareMap.get(DcMotorEx.class, "bearing");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        turnAngle = hardwareMap.get(Servo.class, "turnAngle");
        flicker = hardwareMap.get(Servo.class, "flicker");

        // PIDF Config (40, 0, 0, 15)
        PIDFCoefficients flyPIDF = new PIDFCoefficients(40, 0, 0, 15);
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flyPIDF);
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flyPIDF);

        ticksPerDegBig = GEAR_RATIO_TURRET * (TICKS_PER_MOTOR_REV / 360.0);
        try {
            // bearingZeroOffsetTicks = bearingfirst.bearingpose;
            bearingZeroOffsetTicks = bearing.getCurrentPosition(); // Fallback
        } catch (Exception e) {
            bearingZeroOffsetTicks = bearing.getCurrentPosition();
        }
        bearing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flicker.setPosition(1);

        telemetry.addLine("TUNING MODE READY");
        telemetry.addLine("GP2 Dpad Up/Down: Adjust Velocity");
        telemetry.addLine("GP2 X: Toggle Spin | GP2 O: Shoot");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // --- 1. SENSOR FUSION (Blue Alliance Logic Only) ---
        pinpoint.update();
        follower.update();
        Pose pedroPose = follower.getPose();
        double accurateHeading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
        Pose fusedPose = new Pose(pedroPose.getX(), pedroPose.getY(), accurateHeading);

        // --- 2. DRIVE LOGIC (Blue Perspective: 180 Offset) ---
        driveFieldCentric(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                fusedPose.getHeading(),
                Math.PI // Blue Alliance Fixed
        );

        // --- 3. TUNING CONTROLS (Gamepad 2) ---

        // Adjust Velocity
        if (gamepad2.dpad_up && !lastDpadUp) targetVelocity += 50;
        if (gamepad2.dpad_down && !lastDpadDown) targetVelocity -= 50;
        lastDpadUp = gamepad2.dpad_up;
        lastDpadDown = gamepad2.dpad_down;

        // Toggle Flywheel
        if (gamepad2.x && !lastX) isFlywheelRunning = !isFlywheelRunning;
        lastX = gamepad2.x;

        // Shoot (Intake)
        intake.setPower(gamepad2.circle ? 1.0 : 0.0);

        // Apply Flywheel Power
        if (isFlywheelRunning) {
            flywheel1.setVelocity(targetVelocity);
            flywheel2.setVelocity(targetVelocity);
        } else {
            flywheel1.setPower(0);
            flywheel2.setPower(0);
        }

        // --- 4. AUTO AIM & HOOD LOGIC ---
        // Calculate Distance
        double hRad = fusedPose.getHeading();
        double sensorOffsetX = 3.5;
        double turretX = fusedPose.getX() + sensorOffsetX * Math.cos(hRad);
        double turretY = fusedPose.getY() + sensorOffsetX * Math.sin(hRad);

        double dx = Math.abs(BLUE_GOAL_X - turretX);
        double dy = Math.abs(BLUE_GOAL_Y - turretY);
        double dist = Math.hypot(dx, dy);

        // Hood Logic (Copied from your code)
        double hoodPose = 1;
        if (dist >= 120) {
            hoodPose = 0.4;
        } else if (dist <= 24) {
            hoodPose = 1;
        } else {
            hoodPose = (dist - 24) / 96 * (-0.6) + 1;
        }
        turnAngle.setPosition(hoodPose);

        // Turret Aiming
        if (gamepad2.left_bumper) {
            bearing.setPower(0.5 * gamepad2.left_stick_x); // Manual
        } else {
            autoAimBlue(BLUE_GOAL_X, BLUE_GOAL_Y, fusedPose);
        }

        // --- 5. DATA LOGGING ---
        // Save data point when Square is pressed
        if (gamepad2.square && !lastSquare) {
            String log = String.format("Dist: %.1f | Vel: %.0f | Hood: %.2f", dist, targetVelocity, hoodPose);
            dataPoints.add(log);
        }
        lastSquare = gamepad2.square;

        // --- 6. TELEMETRY ---
        telemetry.addLine("=== TUNING DASHBOARD ===");
        telemetry.addData("DISTANCE TO GOAL", "%.1f inches", dist);
        telemetry.addData("TARGET VELOCITY", "%.0f", targetVelocity);
        telemetry.addData("ACTUAL VELOCITY", "%.0f", flywheel1.getVelocity());
        telemetry.addData("HOOD POS", "%.2f", hoodPose);
        telemetry.addLine("\n--- SAVED POINTS ---");
        for (String p : dataPoints) {
            telemetry.addLine(p);
        }
        telemetry.update();
    }

    private void driveFieldCentric(double strafe, double forward, double turn, double robotHeading, double perspectiveOffset) {
        double y = -forward;
        double x = strafe * 1.1;
        double rx = turn;
        double normalizedHeading = AngleUnit.normalizeRadians(robotHeading - perspectiveOffset);
        double rotX = x * Math.cos(-normalizedHeading) - y * Math.sin(-normalizedHeading);
        double rotY = x * Math.sin(-normalizedHeading) + y * Math.cos(-normalizedHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        leftFront.setPower((rotY + rotX + rx) / denominator * 0.85);
        leftRear.setPower((rotY - rotX + rx) / denominator * 0.85);
        rightFront.setPower((rotY - rotX - rx) / denominator * 0.85);
        rightRear.setPower((rotY + rotX - rx) / denominator * 0.85);
    }

    private void autoAimBlue(double goalX, double goalY, Pose robotPose) {
        double hRad = robotPose.getHeading();
        double sensorOffsetX = 3.5;
        double turretX = robotPose.getX() + sensorOffsetX * Math.cos(hRad);
        double turretY = robotPose.getY() + sensorOffsetX * Math.sin(hRad);
        double dx = Math.abs(goalX - turretX);
        double dy = Math.abs(goalY - turretY);
        double angleToGoalRad = Math.atan2(dx, dy);

        // Blue Alliance Logic Fixed: +90 degrees
        double angleToGoalDeg = Math.toDegrees(angleToGoalRad) + 90;

        Vector vel = follower.getVelocity();
        double deltaTheta = AngleUnit.normalizeRadians(Math.atan2(vel.getYComponent(), vel.getXComponent()) - angleToGoalRad);
        double vTangential = vel.getMagnitude() * Math.sin(deltaTheta);

        double robotHeadingDeg = Math.toDegrees(hRad);
        if(robotHeadingDeg < 0) robotHeadingDeg += 360;
        if(robotHeadingDeg >= 360) robotHeadingDeg %= 360;

        double rawDesiredAngle = wrapDeg(robotHeadingDeg - angleToGoalDeg + (vTangential * PREDICT_FACTOR));

        filterBuffer.add(rawDesiredAngle);
        if (filterBuffer.size() > FILTER_WINDOW_SIZE) filterBuffer.remove();
        double smoothedTarget = 0;
        for (Double val : filterBuffer) smoothedTarget += val;
        smoothedTarget /= filterBuffer.size();

        telemetryBearingTarget = Math.max(TURRET_MIN_DEG, Math.min(TURRET_MAX_DEG, smoothedTarget));
        telemetryBearingCurrent = (bearing.getCurrentPosition() - bearingZeroOffsetTicks) / ticksPerDegBig;
        double error = telemetryBearingTarget - telemetryBearingCurrent;

        if (Math.abs(error) < 1.0) { bearing.setPower(0); lastError = error; return; }
        double power = (error * kP) + ((error - lastError) * kD);
        power = Math.max(-1.0, Math.min(1.0, power));
        lastError = error;

        if (telemetryBearingCurrent > TURRET_MAX_DEG && power * MOTOR_SIGN > 0) power = 0;
        else if (telemetryBearingCurrent < TURRET_MIN_DEG && power * MOTOR_SIGN < 0) power = 0;
        bearing.setPower(power * MOTOR_SIGN);
    }

    private double wrapDeg(double deg) { double d = deg % 360.0; if (d <= -180.0) d += 360.0; if (d > 180.0) d -= 360.0; return d; }
}