package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * ══════════════════════════════════════════════════════════════════════════════
 *  FLYWHEEL PIDF TUNER  —  GoBilda 6000 RPM  |  FTC DECODE 2025-26
 *
 *  Uses RUN_USING_ENCODER + setPIDFCoefficients() + setVelocity().
 *  The REV Hub runs the PIDF loop internally at ~1kHz.
 *
 *  HOW IT WORKS:
 *    setVelocity(target) → REV Hub runs:
 *    power = kF×target + kP×error + kI×∫error + kD×(Δerror/Δt)
 *
 * ══════════════════════════════════════════════════════════════════════════════
 *
 *  ┌─────────────────────────────────────────────────────────────────────────┐
 *  │  TUNING GUIDE                                                           │
 *  ├─────────────────────────────────────────────────────────────────────────┤
 *  │                                                                         │
 *  │  STEP 1 — Tune kF first  (feedforward, the most important value)       │
 *  │    kF scales with your target velocity inside the REV Hub controller.  │
 *  │    Formula starting point:  kF = 32767 ÷ maxTicksPerSec               │
 *  │    For GoBilda 6000 RPM (max ~2800 t/s):  kF ≈ 11.7                   │
 *  │                                                                         │
 *  │    • Press SQUARE to reset, then X to spin flywheels                   │
 *  │    • Hold LEFT TRIGGER + DPAD UP to raise kF                           │
 *  │    • Goal: actual velocity reaches target on its own (~within 150 t/s) │
 *  │    • If it overshoots and oscillates → lower kF slightly               │
 *  │    • Good range: kF ≈ 8 – 15                                           │
 *  │                                                                         │
 *  │  STEP 2 — Tune kP  (closes the remaining gap)                          │
 *  │    • Hold LEFT BUMPER + DPAD UP to raise kP                            │
 *  │    • Watch the error on telemetry drop toward 0                        │
 *  │    • Stop when you see oscillation, then back off 1-2 presses          │
 *  │    • Good range: kP ≈ 20 – 60                                          │
 *  │                                                                         │
 *  │  STEP 3 — Leave kI = 0  (do NOT touch)                                │
 *  │    Integral windup causes dangerous oscillation on flywheels.          │
 *  │                                                                         │
 *  │  STEP 4 — kD is optional                                               │
 *  │    Only add kD if velocity hunts after tuning kF and kP.              │
 *  │    • Hold RIGHT BUMPER + DPAD UP, very small steps                     │
 *  │    • Good range: kD ≈ 0.001 – 0.01                                    │
 *  │                                                                         │
 *  │  STEP 5 — Test across velocity range                                   │
 *  │    Change target with DPAD and verify tracking at 1150–1750 t/s.      │
 *  │                                                                         │
 *  │  NOTE: This uses the REV Hub's internal PIDF, which is separate from  │
 *  │  the FTCLib software PID in your TeleOp/Auto. These values are NOT     │
 *  │  directly transferable to flyP/flyF in your robot code.                │
 *  └─────────────────────────────────────────────────────────────────────────┘
 *
 *  GAMEPAD 2 CONTROLS:
 *    DPAD UP / DOWN               → Target velocity ± 50 ticks/sec
 *    LEFT TRIGGER  + DPAD UP/DOWN → kF ± 0.5
 *    LEFT BUMPER   + DPAD UP/DOWN → kP ± 1.0
 *    RIGHT BUMPER  + DPAD UP/DOWN → kD ± 0.001
 *    RIGHT TRIGGER                → Run intake
 *    X                            → Toggle flywheels ON / OFF
 *    SQUARE                       → Reset all values to defaults
 *
 *  Values apply LIVE every loop — no confirm button needed.
 * ══════════════════════════════════════════════════════════════════════════════
 */
@TeleOp(name = "TUNER: Flywheel PIDF", group = "Tuning")
public class FlywheelPIDF_Tuner extends OpMode {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private DcMotorEx flywheel1, flywheel2, intake;

    // ── PIDF Gains ────────────────────────────────────────────────────────────
    // kF starting point: 32767 / 2800 (max ticks/sec for GoBilda 6000 RPM) ≈ 11.7
    private double kP = 130.0;
    private double kI = 0.0;   // Keep at 0 — DO NOT change
    private double kD = 0.0;
    private double kF = 12.7;  // Tune this FIRST

    // ── Target velocity (ticks/sec) ───────────────────────────────────────────
    // GoBilda 6000 RPM: 28 ticks/rev → max ~2800 ticks/sec
    private double targetVelocity = 1300.0;

    // ── State ─────────────────────────────────────────────────────────────────
    private boolean flywheelsOn = false;

    // ── Button edge detection ─────────────────────────────────────────────────
    private boolean lastDpadUp   = false;
    private boolean lastDpadDown = false;
    private boolean lastX        = false;
    private boolean lastSquare   = false;

    // ── Step sizes ────────────────────────────────────────────────────────────
    private static final double STEP_VELOCITY = 50.0;
    private static final double STEP_KF       = 0.5;
    private static final double STEP_KP       = 1.0;
    private static final double STEP_KD       = 0.001;

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void init() {
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        intake    = hardwareMap.get(DcMotorEx.class, "intake");



        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Apply initial PIDF
        applyPIDF();

        telemetry.addLine("FLYWHEEL PIDF TUNER READY");
        telemetry.addLine("X (GP2) to toggle flywheels.");
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void loop() {
        boolean dpadUp       = gamepad2.dpad_up;
        boolean dpadDown     = gamepad2.dpad_down;
        boolean dpadEdgeUp   = dpadUp   && !lastDpadUp;
        boolean dpadEdgeDown = dpadDown && !lastDpadDown;

        boolean leftTrigger = gamepad2.left_trigger > 0.5;
        boolean leftBumper  = gamepad2.left_bumper;
        boolean rightBumper = gamepad2.right_bumper;

        // ── Adjust target velocity ────────────────────────────────────────────
        if (!leftTrigger && !leftBumper && !rightBumper) {
            if (dpadEdgeUp)   targetVelocity += STEP_VELOCITY;
            if (dpadEdgeDown) targetVelocity = Math.max(0, targetVelocity - STEP_VELOCITY);
        }

        // ── Adjust kF (Left Trigger + Dpad) ──────────────────────────────────
        if (leftTrigger) {
            if (dpadEdgeUp)   kF = Math.max(0, kF + STEP_KF);
            if (dpadEdgeDown) kF = Math.max(0, kF - STEP_KF);
        }

        // ── Adjust kP (Left Bumper + Dpad) ───────────────────────────────────
        if (leftBumper) {
            if (dpadEdgeUp)   kP = Math.max(0, kP + STEP_KP);
            if (dpadEdgeDown) kP = Math.max(0, kP - STEP_KP);
        }

        // ── Adjust kD (Right Bumper + Dpad) ──────────────────────────────────
        if (rightBumper) {
            if (dpadEdgeUp)   kD = Math.max(0, kD + STEP_KD);
            if (dpadEdgeDown) kD = Math.max(0, kD - STEP_KD);
        }

        // ── Toggle flywheels (X) ──────────────────────────────────────────────
        if (gamepad2.x && !lastX) flywheelsOn = !flywheelsOn;
        lastX = gamepad2.x;

        // ── Reset to defaults (Square) ────────────────────────────────────────
        if (gamepad2.square && !lastSquare) {
            kP = 130.0; kI = 0.0; kD = 0.0; kF = 12.7;
            targetVelocity = 1300.0;
        }
        lastSquare = gamepad2.square;

        // ── Apply PIDF live every loop ────────────────────────────────────────
        applyPIDF();

        // ── Drive flywheels ───────────────────────────────────────────────────
        if (flywheelsOn) {
            flywheel1.setVelocity(targetVelocity);
            flywheel2.setVelocity(targetVelocity);
        } else {
            flywheel1.setPower(0);
            flywheel2.setPower(0);
        }

        // ── Intake (Right Trigger) ────────────────────────────────────────────
        intake.setPower(-gamepad2.right_trigger);

        // ── Telemetry ─────────────────────────────────────────────────────────
        double fw1Vel = flywheel1.getVelocity();
        double fw2Vel = flywheel2.getVelocity();
        double fw1Err = targetVelocity - fw1Vel;
        double fw2Err = targetVelocity - fw2Vel;
        boolean fw1Ready = flywheelsOn && Math.abs(fw1Err) <= 80;
        boolean fw2Ready = flywheelsOn && Math.abs(fw2Err) <= 80;

        telemetry.addLine("══════  FLYWHEEL PIDF TUNER  ══════");
        telemetry.addLine("");
        telemetry.addData("Flywheels",  flywheelsOn ? "▶  ON" : "■  OFF");
        telemetry.addData("Target Vel", "%.0f  ticks/sec", targetVelocity);
        telemetry.addLine("");
        telemetry.addLine("── Actual Velocities ────────────────────");
        telemetry.addData("FW1", "%.0f t/s   err: %+.0f  %s", fw1Vel, fw1Err, fw1Ready ? "✓ READY" : "");
        telemetry.addData("FW2", "%.0f t/s   err: %+.0f  %s", fw2Vel, fw2Err, fw2Ready ? "✓ READY" : "");
        telemetry.addLine("");
        telemetry.addLine("── PIDF Gains ───────────────────────────");
        telemetry.addData("kF  (L.TRIG + DPAD)", "%.1f",  kF);
        telemetry.addData("kP  (L.BUMP + DPAD)", "%.1f",  kP);
        telemetry.addData("kI  (keep = 0!)",      "%.3f", kI);
        telemetry.addData("kD  (R.BUMP + DPAD)", "%.3f",  kD);
        telemetry.addLine("");
        telemetry.addLine("── Controls ─────────────────────────────");
        telemetry.addLine("DPAD U/D        → Velocity ± 50");
        telemetry.addLine("L.TRIG + DPAD   → kF ± 0.5");
        telemetry.addLine("L.BUMP + DPAD   → kP ± 1.0");
        telemetry.addLine("R.BUMP + DPAD   → kD ± 0.001");
        telemetry.addLine("R.TRIG          → Intake");
        telemetry.addLine("X               → Toggle flywheels");
        telemetry.addLine("SQUARE          → Reset to defaults");
        telemetry.update();

        // Edge tracking
        lastDpadUp   = dpadUp;
        lastDpadDown = dpadDown;
    }

    // ─────────────────────────────────────────────────────────────────────────
    private void applyPIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, kF);
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }
}
