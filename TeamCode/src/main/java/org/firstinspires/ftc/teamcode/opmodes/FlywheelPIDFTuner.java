package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelPIDF;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Tune the flywheel velocity controller (FlywheelPIDF) live from gamepad 1.
 * The robot does not drive. Only the flywheel spins (and the feeder while B is held).
 *
 *   A (cross)           spin on / off
 *   DPAD UP / DOWN      target velocity +/- 50 ticks/s
 *   DPAD LEFT / RIGHT   select gain: kF, kP, kI, kD
 *   RB / LB             selected gain + / - one step   (hold RIGHT TRIGGER for 10x steps)
 *   Y (triangle)        set the selected gain to 0
 *   B (circle)          HOLD to feed a ball (gate open + intake), to test recovery
 *
 * Copy the final gains into subsystems/FlywheelPIDF.java (shown at the bottom of telemetry).
 */
@TeleOp(name = "TUNE: Flywheel PIDF", group = "Tuning")
public class FlywheelPIDFTuner extends OpMode {

    private static final String[] NAMES = { "kF", "kP", "kI", "kD" };
    private static final double[] STEPS = { 0.00001, 0.0001, 0.00001, 0.00001 };

    private DcMotorEx flywheel, intakeLeft, intakeRight;
    private Servo gate;
    private final FlywheelPIDF pidf = new FlywheelPIDF();

    private boolean spinning = false;
    private double target = 1300;
    private int selected = 0;

    // Measurements
    private final ElapsedTime sinceChange = new ElapsedTime();
    private double riseTime = -1;      // seconds from spin-up / target change to within tolerance
    private double maxOvershoot = 0;   // ticks/s above target after first reaching it
    private double avgAbsError = 0;    // smoothed |error|
    private double minDuringFeed = Double.NaN;

    private boolean lastA, lastUp, lastDown, lastLeft, lastRight, lastRB, lastLB, lastY;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(Shooter.FLYWHEEL_DIR);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setPower(0);

        intakeLeft  = hardwareMap.tryGet(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.tryGet(DcMotorEx.class, "intakeRight");
        if (intakeLeft != null)  intakeLeft.setDirection(Shooter.INTAKE_LEFT_DIR);
        if (intakeRight != null) intakeRight.setDirection(Shooter.INTAKE_RIGHT_DIR);
        gate = hardwareMap.tryGet(Servo.class, "gate");
        if (gate != null) gate.setPosition(Shooter.GATE_CLOSE);
    }

    @Override
    public void loop() {
        // ── Buttons ──────────────────────────────────────────────────────────
        if (gamepad1.a && !lastA) { spinning = !spinning; restartMeasurements(); }
        if (gamepad1.dpad_up && !lastUp)     { target += 50; restartMeasurements(); }
        if (gamepad1.dpad_down && !lastDown) { target = Math.max(0, target - 50); restartMeasurements(); }
        if (gamepad1.dpad_right && !lastRight) selected = (selected + 1) % NAMES.length;
        if (gamepad1.dpad_left && !lastLeft)   selected = (selected + NAMES.length - 1) % NAMES.length;

        double step = STEPS[selected] * (gamepad1.right_trigger > 0.5 ? 10 : 1);
        if (gamepad1.right_bumper && !lastRB) setGain(selected, getGain(selected) + step);
        if (gamepad1.left_bumper && !lastLB)  setGain(selected, Math.max(0, getGain(selected) - step));
        if (gamepad1.triangle && !lastY)      setGain(selected, 0);

        lastA = gamepad1.a; lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;
        lastRB = gamepad1.right_bumper; lastLB = gamepad1.left_bumper; lastY = gamepad1.triangle;

        // ── Flywheel ─────────────────────────────────────────────────────────
        double vel = flywheel.getVelocity();
        double power = 0;
        if (spinning) {
            power = pidf.calculate(target, vel);
        } else {
            pidf.reset();
        }
        flywheel.setPower(power);

        // ── Feed (B held) ────────────────────────────────────────────────────
        boolean feeding = gamepad1.circle && spinning;
        double feedPower = feeding ? 1.0 : 0;
        if (intakeLeft != null)  intakeLeft.setPower(feedPower);
        if (intakeRight != null) intakeRight.setPower(feedPower);
        if (gate != null) gate.setPosition(feeding ? Shooter.GATE_OPEN : Shooter.GATE_CLOSE);
        if (feeding) minDuringFeed = Double.isNaN(minDuringFeed) ? vel : Math.min(minDuringFeed, vel);

        // ── Measurements ─────────────────────────────────────────────────────
        double error = target - vel;
        if (spinning) {
            if (riseTime < 0 && Math.abs(error) < Shooter.AT_SPEED_TOLERANCE) riseTime = sinceChange.seconds();
            if (riseTime >= 0) maxOvershoot = Math.max(maxOvershoot, -error);
            avgAbsError = 0.95 * avgAbsError + 0.05 * Math.abs(error);
        }

        // ── Telemetry ────────────────────────────────────────────────────────
        telemetry.addData("Spinning", spinning ? "ON  (A to stop)" : "off (A to start)");
        telemetry.addData("Target", "%.0f ticks/s", target);
        telemetry.addData("Actual", "%.0f   error %+.0f", vel, error);
        telemetry.addData("Power", "%.3f", power);
        telemetry.addLine();
        telemetry.addData("Rise time", riseTime < 0 ? "-" : String.format("%.2f s", riseTime));
        telemetry.addData("Overshoot", "%.0f ticks/s", maxOvershoot);
        telemetry.addData("Avg |error|", "%.0f ticks/s", avgAbsError);
        telemetry.addData("Lowest while feeding", Double.isNaN(minDuringFeed) ? "-" : String.format("%.0f", minDuringFeed));
        telemetry.addLine();
        for (int i = 0; i < NAMES.length; i++) {
            telemetry.addData((i == selected ? ">> " : "   ") + NAMES[i], "%.6f", getGain(i));
        }
        telemetry.addData("Step", "%.6f%s", step, gamepad1.right_trigger > 0.5 ? " (x10)" : "");
        telemetry.addLine();
        telemetry.addLine(String.format("KF = %.6f; KP = %.6f; KI = %.6f; KD = %.6f;",
                FlywheelPIDF.KF, FlywheelPIDF.KP, FlywheelPIDF.KI, FlywheelPIDF.KD));
        telemetry.addLine("A spin | DPAD U/D target | DPAD L/R gain | RB/LB +/- | RT x10 | Y zero | B feed");
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.setPower(0);
        if (intakeLeft != null)  intakeLeft.setPower(0);
        if (intakeRight != null) intakeRight.setPower(0);
    }

    private void restartMeasurements() {
        sinceChange.reset();
        riseTime = -1;
        maxOvershoot = 0;
        minDuringFeed = Double.NaN;
    }

    private static double getGain(int i) {
        switch (i) {
            case 0: return FlywheelPIDF.KF;
            case 1: return FlywheelPIDF.KP;
            case 2: return FlywheelPIDF.KI;
            default: return FlywheelPIDF.KD;
        }
    }

    private static void setGain(int i, double v) {
        switch (i) {
            case 0: FlywheelPIDF.KF = v; break;
            case 1: FlywheelPIDF.KP = v; break;
            case 2: FlywheelPIDF.KI = v; break;
            default: FlywheelPIDF.KD = v; break;
        }
    }
}
