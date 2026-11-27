package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Flywheel encoder check: prints getVelocity() (ticks/s).
 * Spin the wheel by hand, or hold RIGHT TRIGGER to drive it (power = trigger).
 * If velocity stays 0 while the wheel spins, the encoder cable is unplugged or in the wrong port.
 */
@TeleOp(name = "TEST: Flywheel Encoder", group = "Test")
public class FlywheelEncoderTest extends OpMode {
    private DcMotorEx flywheel;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(Shooter.FLYWHEEL_DIR); // same direction as the TeleOp
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        flywheel.setPower(gamepad1.right_trigger);

        telemetry.addData("Velocity (ticks/s)", "%.0f", flywheel.getVelocity());
        telemetry.addData("Position (ticks)", flywheel.getCurrentPosition());
        telemetry.addData("Power", "%.2f", gamepad1.right_trigger);
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.setPower(0);
    }
}
