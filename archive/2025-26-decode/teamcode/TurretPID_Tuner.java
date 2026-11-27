package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * ══════════════════════════════════════════════════════════════════════════════
 *  TURRET PID TUNER  —  GoBilda 435 RPM  |  125:33 gear ratio  |  FTC DECODE
 *
 *  Holds the turret at a chosen target angle using a PD+I controller.
 *  All gains and the tolerance apply directly to RedAllianceTeleop and
 *  MyRobotTeleopMecanumFieldRelativeDrive — copy the values once happy.
 *
 * ══════════════════════════════════════════════════════════════════════════════
 *
 *  ┌─────────────────────────────────────────────────────────────────────────┐
 *  │  TUNING GUIDE                                                           │
 *  ├─────────────────────────────────────────────────────────────────────────┤
 *  │                                                                         │
 *  │  STEP 1 — Zero the turret                                               │
 *  │    Point turret straight forward, then press X to reset the encoder.    │
 *  │    Target stays at 0°; the turret should hold position.                 │
 *  │                                                                         │
 *  │  STEP 2 — Tune kP  (main correction force)                             │
 *  │    • Hold LEFT BUMPER + DPAD UP to raise kP.                            │
 *  │    • Set target to +45° (DPAD UP, no modifier).                        │
 *  │    • Raise kP until the turret reaches 45° briskly.                     │
 *  │    • If it oscillates (rocks back and forth), lower kP slightly.        │
 *  │    • Good starting range: 0.01 – 0.1                                   │
 *  │                                                                         │
 *  │  STEP 3 — Tune kD  (dampen oscillation)                                │
 *  │    • Only add kD if you see oscillation at the target.                 │
 *  │    • Hold RIGHT BUMPER + DPAD UP to raise kD in small steps.           │
 *  │    • Stop when oscillation disappears; too much → sluggish response.   │
 *  │    • Good starting range: 0.00001 – 0.001                              │
 *  │                                                                         │
 *  │  STEP 4 — Tune kI  (eliminate steady-state offset)                     │
 *  │    • Watch "Error" on telemetry after turret settles.                   │
 *  │    • If a small constant offset remains (e.g. −0.5°), add a tiny kI.  │
 *  │    • Hold LEFT TRIGGER + DPAD UP to raise kI very slowly.              │
 *  │    • Stop as soon as the offset disappears — kI causes windup if       │
 *  │      too large. Keep it below 0.0001.                                  │
 *  │                                                                         │
 *  │  STEP 5 — Set Tolerance                                                 │
 *  │    • Tolerance is the dead-band (°) where the motor cuts to zero.      │
 *  │    • Hold RIGHT TRIGGER + DPAD DOWN to shrink it.                      │
 *  │    • Go as low as you can without the turret hunting (jittering).      │
 *  │    • Typical good value: 0.2° – 0.5°                                  │
 *  │                                                                         │
 *  │  STEP 6 — Test across the range                                         │
 *  │    • Sweep DPAD UP through 0° → 90° → 180° → 270°.                   │
 *  │    • Then sweep back through 0° → −90° → −120°.                       │
 *  │    • Verify the turret tracks smoothly and settles within tolerance.    │
 *  │                                                                         │
 *  │  STEP 7 — Copy values to teleop files                                  │
 *  │    RedAllianceTeleop:                   TURRET_KP / KI / KD / TOLERANCE│
 *  │    MyRobotTeleopMecanumFieldRelative:   kP / kI / kD / TURRET_TOLERANCE│
 *  │                                                                         │
 *  └─────────────────────────────────────────────────────────────────────────┘
 *
 *  GAMEPAD 1 CONTROLS:
 *    RIGHT BUMPER                 → Toggle target 0° ↔ 45° (repeatability test)
 *    DPAD UP / DOWN               → Target angle ± 5°
 *    LEFT BUMPER  + DPAD UP/DOWN  → kP ± 0.005
 *    Y            + DPAD UP/DOWN  → kD ± 0.000005
 *    LEFT TRIGGER + DPAD UP/DOWN  → kI ± 0.000001
 *    RIGHT TRIGGER+ DPAD UP/DOWN  → Tolerance ± 0.1°
 *    X                            → Reset encoder (zero at current position)
 *    Square                       → Reset all values to defaults
 * ══════════════════════════════════════════════════════════════════════════════
 */
@TeleOp(name = "TUNER: Turret PID", group = "Tuning")
public class TurretPID_Tuner extends OpMode {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private DcMotorEx bearing;

    // ── Turret constants (must match teleop) ──────────────────────────────────
    private static final double TICKS_PER_MOTOR_REV = 383.6;
    private static final double TURRET_GEAR_RATIO    = 125.0 / 33.0;
    private static final double TICKS_PER_DEG        = TICKS_PER_MOTOR_REV * TURRET_GEAR_RATIO / 360.0;
    private static final double TURRET_MAX_DEG       =  270.0;
    private static final double TURRET_MIN_DEG       = -120.0;

    // ── Tunable gains ─────────────────────────────────────────────────────────
    private double kP        = 0.05;
    private double kI        = 0.0;
    private double kD        = 0.00005;
    private double tolerance = 0.1;   // degrees — deadband where motor cuts to 0

    // ── Target ────────────────────────────────────────────────────────────────
    private double targetDeg = 0.0;

    // ── PD state ──────────────────────────────────────────────────────────────
    private double lastError   = 0;
    private double integrator  = 0;

    // ── Step sizes ────────────────────────────────────────────────────────────
    private static final double STEP_TARGET = 5.0;
    private static final double STEP_KP     = 0.005;
    private static final double STEP_KI     = 0.000001;
    private static final double STEP_KD     = 0.000005;
    private static final double STEP_TOL    = 0.1;

    // ── Toggle state ─────────────────────────────────────────────────────────
    private boolean toggleAt45 = false;   // false = 0°, true = 45°

    // ── Button edge detection ─────────────────────────────────────────────────
    private boolean lastDpadUp   = false;
    private boolean lastDpadDown = false;
    private boolean lastX        = false;
    private boolean lastSquare   = false;
    private boolean lastRB       = false;
    private boolean lastY        = false;

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void init() {
        bearing = hardwareMap.get(DcMotorEx.class, "bearing");
        bearing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bearing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bearing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bearing.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("TURRET PID TUNER READY");
        telemetry.addLine("Point turret forward, press X to zero, then tune.");
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void loop() {
        // ── Edge detection ────────────────────────────────────────────────────
        boolean dpadUp   = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean edgeUp   = dpadUp   && !lastDpadUp;
        boolean edgeDown = dpadDown && !lastDpadDown;
        lastDpadUp   = dpadUp;
        lastDpadDown = dpadDown;

        boolean leftBumper  = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;
        boolean yBtn        = gamepad1.y;
        boolean leftTrig    = gamepad1.left_trigger  > 0.5;
        boolean rightTrig   = gamepad1.right_trigger > 0.5;

        // ── RB: toggle target 0° ↔ 45° ───────────────────────────────────────
        if (rightBumper && !lastRB) {
            toggleAt45 = !toggleAt45;
            targetDeg  = toggleAt45 ? 45.0 : 0.0;
            integrator = 0;
            lastError  = 0;
        }
        lastRB = rightBumper;

        // ── Adjust gains / target ─────────────────────────────────────────────
        if (leftBumper) {
            // kP
            if (edgeUp)   kP = Math.max(0, kP + STEP_KP);
            if (edgeDown) kP = Math.max(0, kP - STEP_KP);
        } else if (yBtn) {
            // kD
            if (edgeUp)   kD = Math.max(0, kD + STEP_KD);
            if (edgeDown) kD = Math.max(0, kD - STEP_KD);
        } else if (leftTrig) {
            // kI
            if (edgeUp)   kI = Math.max(0, kI + STEP_KI);
            if (edgeDown) kI = Math.max(0, kI - STEP_KI);
        } else if (rightTrig) {
            // Tolerance
            if (edgeUp)   tolerance = Math.max(0.05, tolerance + STEP_TOL);
            if (edgeDown) tolerance = Math.max(0.05, tolerance - STEP_TOL);
        } else {
            // Target angle (manual)
            if (edgeUp)   targetDeg = Math.min(TURRET_MAX_DEG, targetDeg + STEP_TARGET);
            if (edgeDown) targetDeg = Math.max(TURRET_MIN_DEG, targetDeg - STEP_TARGET);
        }

        // ── Reset encoder (X) ─────────────────────────────────────────────────
        if (gamepad1.x && !lastX) {
            bearing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bearing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            integrator  = 0;
            lastError   = 0;
            targetDeg   = 0;
        }
        lastX = gamepad1.x;

        // ── Reset all to defaults (Square) ────────────────────────────────────
        if (gamepad1.square && !lastSquare) {
            kP = 0.05; kI = 0.0; kD = 0.00005; tolerance = 0.1; targetDeg = 0;
            toggleAt45 = false; integrator = 0; lastError = 0;
        }
        lastSquare = gamepad1.square;

        // ── PID control ───────────────────────────────────────────────────────
        double currentAngle = bearing.getCurrentPosition() / TICKS_PER_DEG;
        double error        = targetDeg - currentAngle;

        double power = 0;
        if (Math.abs(error) >= tolerance) {
            integrator += error;                          // accumulate
            integrator  = Math.max(-50, Math.min(50, integrator)); // anti-windup clamp

            double p = kP * error;
            double i = kI * integrator;
            double d = kD * (error - lastError);
            power = Math.max(-1.0, Math.min(1.0, p + i + d));
            lastError = error;

            // Hard stops at physical limits
            if      (currentAngle > TURRET_MAX_DEG && power > 0) power = 0;
            else if (currentAngle < TURRET_MIN_DEG && power < 0) power = 0;
        } else {
            integrator = 0;  // reset integrator inside dead-band so it doesn't windup
            lastError  = 0;
        }

        bearing.setPower(power);

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addLine("══════  TURRET PID TUNER  ══════");
        telemetry.addLine("");
        telemetry.addLine("── Position ──────────────────────────");
        telemetry.addData("Current Angle", "%.2f °", currentAngle);
        telemetry.addData("Target  Angle", "%.1f °  [%s]", targetDeg, toggleAt45 ? "45°" : " 0°");
        telemetry.addData("Error",         "%+.3f °", error);
        telemetry.addData("Settled",       Math.abs(error) < tolerance ? "✓  YES" : "   ...");
        telemetry.addData("Motor Power",   "%.3f", power);
        telemetry.addLine("");
        telemetry.addLine("── Gains ─────────────────────────────");
        telemetry.addData("kP  (LB + DPAD)",  "%.5f",  kP);
        telemetry.addData("kI  (LT + DPAD)",  "%.6f",  kI);
        telemetry.addData("kD  (Y  + DPAD)",  "%.6f",  kD);
        telemetry.addData("Tol (RT + DPAD)",  "%.3f °", tolerance);
        telemetry.addLine("");
        telemetry.addLine("── Controls ──────────────────────────");
        telemetry.addLine("RB                → Toggle 0° ↔ 45°");
        telemetry.addLine("DPAD U/D          → Target ± 5°");
        telemetry.addLine("LB  + DPAD U/D    → kP  ± 0.005");
        telemetry.addLine("Y   + DPAD U/D    → kD  ± 0.000005");
        telemetry.addLine("L.T + DPAD U/D    → kI  ± 0.000001");
        telemetry.addLine("R.T + DPAD U/D    → Tol ± 0.1°");
        telemetry.addLine("X                 → Zero encoder");
        telemetry.addLine("SQUARE            → Reset all");
        telemetry.update();
    }
}
