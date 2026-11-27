package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Turret encoder check. The motor is NOT powered, so turn the turret by hand.
 *
 *   1. Point the turret straight forward, then INIT (encoder is zeroed).
 *   2. Press PLAY, rotate the turret exactly 180° CLOCKWISE (seen from above).
 *   3. Read "Ticks". Cross (✕) re-zeroes at any time.
 *
 * Uses the same direction (REVERSE) as the real turret code, so the sign matches what the shooter sees.
 */
@TeleOp(name = "TEST: Turret Ticks", group = "Test")
public class TurretTicksTest extends OpMode {

    // Current values in Shooter.java, to compare against.
    private static final double TICKS_PER_MOTOR_REV = 383.6;
    private static final double GEAR_RATIO          = 125.0 / 33.0;
    private static final double TICKS_PER_DEG       = TICKS_PER_MOTOR_REV * GEAR_RATIO / 360.0;

    private DcMotorEx bearing;

    @Override
    public void init() {
        bearing = hardwareMap.get(DcMotorEx.class, "bearing");
        bearing.setDirection(DcMotorSimple.Direction.REVERSE);
        bearing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bearing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bearing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // free to turn by hand
        bearing.setPower(0);
    }

    @Override
    public void init_loop() {
        showTelemetry();
    }

    @Override
    public void loop() {
        if (gamepad1.cross) {
            bearing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bearing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        bearing.setPower(0);
        showTelemetry();
    }

    private void showTelemetry() {
        int ticks = bearing.getCurrentPosition();
        telemetry.addData("Ticks", ticks);
        telemetry.addData("Angle with current ratio", "%.1f°", ticks / TICKS_PER_DEG);
        telemetry.addData("Expected for 180°", "%.0f ticks", 180 * TICKS_PER_DEG);
        telemetry.addLine();
        telemetry.addLine("Turret forward -> INIT -> PLAY -> rotate 180° clockwise by hand");
        telemetry.addLine("Cross = re-zero");
        telemetry.update();
    }
}
