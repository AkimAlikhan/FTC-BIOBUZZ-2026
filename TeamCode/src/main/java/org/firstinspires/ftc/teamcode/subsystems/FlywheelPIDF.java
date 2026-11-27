package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Flywheel velocity controller, used by the Shooter AND by the Flywheel PIDF tuner,
 * so the gains you tune are exactly what runs in matches.
 *
 *   power = kF * target + kP * error + kI * integral(error) + kD * d(error)/dt
 *   (velocities in ticks/s, power clipped to 0..1)
 *
 * Put the numbers from "TUNE: Flywheel PIDF" here.
 */
public class FlywheelPIDF {
    public static double KF = 0.00039;
    public static double KP = 0.0022;
    public static double KI = 0.0;
    public static double KD = 0.0;

    /** Only build up the integral when this close to the target (ticks/s), to avoid wind-up during spin-up. */
    public static double I_ZONE = 150;

    private double integral = 0;
    private double lastError = 0;
    private boolean first = true;
    private final ElapsedTime timer = new ElapsedTime();

    public double calculate(double targetVel, double currentVel) {
        double error = targetVel - currentVel;
        double dt = first ? 0 : timer.seconds();
        timer.reset();

        if (Math.abs(error) < I_ZONE && dt > 0) integral += error * dt;
        else if (Math.abs(error) >= I_ZONE) integral = 0;

        double derivative = (first || dt <= 0) ? 0 : (error - lastError) / dt;
        lastError = error;
        first = false;

        double power = KF * targetVel + KP * error + KI * integral + KD * derivative;
        return Math.max(0, Math.min(1, power));
    }

    public void reset() {
        integral = 0;
        lastError = 0;
        first = true;
    }
}
