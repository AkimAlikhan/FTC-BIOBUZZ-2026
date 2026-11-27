package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Find servo positions (gate open/close, hood range, ...).
 *
 * GAMEPAD 1
 *   DPAD LEFT / RIGHT   select servo
 *   DPAD UP / DOWN      +/- 0.01  (fine)
 *   RB / LB             +/- 0.05  (coarse)
 *   A                   jump to 0.5
 *
 * Servos don't move until you press PLAY. Each then goes to its START value below.
 * Write the numbers you like into subsystems/Shooter.java (GATE_OPEN, GATE_CLOSE, HOOD_POS).
 * To tune more servos, add their Driver Hub names to NAMES.
 */
@TeleOp(name = "TUNE: Servo Positions", group = "Test")
public class ServoTuner extends OpMode {

    private static final String[] NAMES = { "gate", "turnAngle" };
    private static final double[]  START = { Shooter.GATE_CLOSE, Shooter.HOOD_POS };

    private static final double FINE = 0.01;
    private static final double COARSE = 0.05;

    private final Servo[] servos = new Servo[NAMES.length];
    private final double[] pos = new double[NAMES.length];
    private int selected = 0;

    private boolean lastUp, lastDown, lastLeft, lastRight, lastRB, lastLB, lastA;

    @Override
    public void init() {
        for (int i = 0; i < NAMES.length; i++) {
            servos[i] = hardwareMap.tryGet(Servo.class, NAMES[i]);
            pos[i] = START[i];
        }
        for (int i = 0; i < NAMES.length; i++) {
            telemetry.addData(NAMES[i], servos[i] != null ? "found" : "NOT IN CONFIG (skipped)");
        }
        telemetry.addLine("Press PLAY. Servos move to their start positions.");
        telemetry.update();
    }

    @Override
    public void start() {
        for (int i = 0; i < NAMES.length; i++) {
            if (servos[i] != null) servos[i].setPosition(pos[i]);
        }
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_right && !lastRight) selected = (selected + 1) % NAMES.length;
        if (gamepad1.dpad_left && !lastLeft)   selected = (selected + NAMES.length - 1) % NAMES.length;

        double delta = 0;
        if (gamepad1.dpad_up && !lastUp)          delta += FINE;
        if (gamepad1.dpad_down && !lastDown)      delta -= FINE;
        if (gamepad1.right_bumper && !lastRB)     delta += COARSE;
        if (gamepad1.left_bumper && !lastLB)      delta -= COARSE;
        pos[selected] = Math.max(0, Math.min(1, pos[selected] + delta));
        if (gamepad1.a && !lastA) pos[selected] = 0.5;

        if (servos[selected] != null) servos[selected].setPosition(pos[selected]);

        lastUp = gamepad1.dpad_up;       lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;   lastRight = gamepad1.dpad_right;
        lastRB = gamepad1.right_bumper;  lastLB = gamepad1.left_bumper;
        lastA = gamepad1.a;

        for (int i = 0; i < NAMES.length; i++) {
            String state = servos[i] == null ? "  (not in config)" : "";
            telemetry.addData((i == selected ? ">> " : "   ") + NAMES[i], "%.3f%s", pos[i], state);
        }
        telemetry.addLine();
        telemetry.addLine("DPAD L/R select | DPAD U/D ±0.01 | RB/LB ±0.05 | A = 0.5");
        telemetry.update();
    }
}
