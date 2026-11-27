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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Gamepad Shooter Tuner", group = "Tuning")
public class GamepadShooterTuner extends OpMode {

    // --- HARDWARE ---
    private DcMotorEx flywheel1, flywheel2, intake;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private Servo turnAngle; // Added Hood Servo

    private PIDController controller;

    // --- PEDRO FOLLOWER ---
    private Follower follower;
    private final double BLUE_GOAL_X = 139;
    private final double BLUE_GOAL_Y = 139;

    // --- LOOKUP TABLE DATA (HOOD ONLY) ---
    private static final double[] DISTANCES =       { 40,   50,   60,   70,   80,   90,    100,  110,  120,  130,  140,  150 };
    private static final double[] HOOD_ANGLES =     { 0.0,  0.0,  0.0,  0.2,  0.2,  0.25,  0.35, 0.4,  0.45, 0.5,  0.5,  0.5 };

    private double targetHoodPos = 0;

    // Tuning Variables
    private double p = 0.0, i = 0.0, d = 0.0, f = 0.0;
    private int targetVelocity = 1500;
    private double bangBangThreshold = 100;

    // Interface Variables
    private int selectionIndex = 0;
    private String[] labels = {"P (Proportional)", "I (Integral)", "D (Derivative)", "F (Feedforward)", "Target Velocity"};

    // To handle button presses (debounce)
    private boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

    @Override
    public void init() {
        // 1. Setup Follower and Pose
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));

        // 2. Setup Drive Motors
        leftFront = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        leftRear = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        rightFront = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        rightRear = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 3. Setup Shooter & Servo
        controller = new PIDController(p, i, d);
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turnAngle = hardwareMap.get(Servo.class, "turnAngle"); // Init Servo

        // Important: RUN_WITHOUT_ENCODER so our custom PID works
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Ready. Starting at (72, 72).");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- 1. UPDATE PEDRO ---
        follower.update();
        Pose currentPose = follower.getPose();
        double distToBlue = Math.hypot(BLUE_GOAL_X - currentPose.getX(), BLUE_GOAL_Y - currentPose.getY());

        // --- 2. CALCULATE HOOD ANGLE (AUTOMATIC) ---
        calculateHoodTarget(distToBlue);
        turnAngle.setPosition(targetHoodPos);

        // --- 3. DRIVE LOGIC (Field Centric) ---
        driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, currentPose.getHeading());

        // --- 4. HANDLE TUNING INPUTS ---
        handleGamepadInput();

        // --- 5. UPDATE PID ---
        controller.setPID(p, i, d);

        // --- 6. CALCULATE POWER ---
        double currentVel = flywheel1.getVelocity();

        // Calculate PID
        double pid = controller.calculate(currentVel, targetVelocity);
        double ff = targetVelocity * f;
        double power = pid + ff;

        // Bang-Bang (Full speed if very far behind, optional safety)
        if (targetVelocity - currentVel > bangBangThreshold) {
            // power = 1.0;
        }

        // Safety Clamp
        power = Math.max(-1.0, Math.min(1.0, power));

        // --- 7. APPLY POWER (With Kill Switch) ---
        if (gamepad1.x) {
            // Kill Switch
            flywheel1.setPower(0);
            flywheel2.setPower(0);
        } else {
            // Normal Operation
            flywheel1.setPower(power);
            flywheel2.setPower(power);
        }

        // --- 8. TELEMETRY ---
        telemetry.addLine("=== SHOOTER TUNER ===");
        telemetry.addData("Pos", "X:%.0f Y:%.0f", currentPose.getX(), currentPose.getY());
        telemetry.addData("Dist to Goal", "%.1f", distToBlue);
        telemetry.addData("Hood Angle", "%.2f", targetHoodPos);
        telemetry.addLine("-----------------------------");

        for (int j = 0; j < labels.length; j++) {
            String prefix = (j == selectionIndex) ? ">> " : "   ";
            double val = getValue(j);
            if (j == 4) telemetry.addData(prefix + labels[j], "%.0f", val);
            else        telemetry.addData(prefix + labels[j], "%.5f", val);
        }

        telemetry.addLine("\n=== PERFORMANCE ===");
        telemetry.addData("Target Vel", targetVelocity);
        telemetry.addData("Actual Vel", "%.1f", currentVel);
        telemetry.addData("Error", "%.1f", targetVelocity - currentVel);
        telemetry.addData("Power", "%.2f", power);
        telemetry.update();
    }

    /**
     * Calculates Hood Angle based on Distance using Linear Interpolation
     */
    private void calculateHoodTarget(double dist) {
        if (dist <= DISTANCES[0]) {
            targetHoodPos = HOOD_ANGLES[0];
            return;
        }
        if (dist >= DISTANCES[DISTANCES.length - 1]) {
            targetHoodPos = HOOD_ANGLES[HOOD_ANGLES.length - 1];
            return;
        }
        int index = 0;
        while (index < DISTANCES.length - 1 && DISTANCES[index + 1] < dist) {
            index++;
        }
        double t = (dist - DISTANCES[index]) / (DISTANCES[index + 1] - DISTANCES[index]);
        // Only interpolate Hood Angle, ignore Velocity
        targetHoodPos = HOOD_ANGLES[index] + (t * (HOOD_ANGLES[index + 1] - HOOD_ANGLES[index]));
    }

    private void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, double robotHeadingRad) {
        // Typically strafe and forward need negation depending on joystick mapping
        // strafeSpeed *= -1;
        // forwardSpeed *= -1;

        double rotX = strafeSpeed * Math.cos(-robotHeadingRad) - (-forwardSpeed) * Math.sin(-robotHeadingRad);
        double rotY = strafeSpeed * Math.sin(-robotHeadingRad) + (-forwardSpeed) * Math.cos(-robotHeadingRad);

        double frontLeftPower  = rotY + rotX + turnSpeed;
        double backLeftPower   = rotY - rotX + turnSpeed;
        double frontRightPower = rotY - rotX - turnSpeed;
        double backRightPower  = rotY + rotX - turnSpeed;

        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }

    private void handleGamepadInput() {
        intake.setPower(gamepad1.right_trigger);

        // Navigation (Up/Down)
        if (gamepad1.dpad_up && !lastUp) {
            selectionIndex--;
            if (selectionIndex < 0) selectionIndex = 4;
        }
        lastUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !lastDown) {
            selectionIndex++;
            if (selectionIndex > 4) selectionIndex = 0;
        }
        lastDown = gamepad1.dpad_down;

        // Edit Value (Left/Right)
        double multiplier = 1.0;
        if (gamepad1.right_bumper) multiplier = 10.0;
        if (gamepad1.left_bumper) multiplier = 0.1;

        if (gamepad1.dpad_right && !lastRight) {
            changeValue(selectionIndex, 1 * multiplier);
        }
        lastRight = gamepad1.dpad_right;

        if (gamepad1.dpad_left && !lastLeft) {
            changeValue(selectionIndex, -1 * multiplier);
        }
        lastLeft = gamepad1.dpad_left;
    }

    private double getValue(int index) {
        switch (index) {
            case 0: return p;
            case 1: return i;
            case 2: return d;
            case 3: return f;
            case 4: return targetVelocity;
            default: return 0;
        }
    }

    private void changeValue(int index, double scale) {
        double stepPID = 0.0001;
        double stepTarget = 50;

        switch (index) {
            case 0: p += stepPID * scale; break;
            case 1: i += stepPID * scale; break;
            case 2: d += stepPID * scale; break;
            case 3: f += stepPID * scale; break;
            case 4: targetVelocity += stepTarget * scale; break;
        }

        if (p < 0) p = 0;
        if (i < 0) i = 0;
        if (d < 0) d = 0;
        if (f < 0) f = 0;
    }
}