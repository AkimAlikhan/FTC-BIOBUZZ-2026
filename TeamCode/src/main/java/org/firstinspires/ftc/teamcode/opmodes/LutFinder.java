package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

/**
 * Find LUT values: everything works like "TeleOp" (drive, turret aim, intake, LB shot),
 * but flywheel velocity and hood are set by hand, and you record the ones that score.
 * Always starts at (72, 72, 90°) and never touches the Auto/TeleOp saved pose.
 *
 * GAMEPAD 1 (on top of the normal TeleOp controls)
 *   DPAD UP / DOWN      flywheel velocity +/- 25 ticks/s
 *   DPAD LEFT / RIGHT   hood -/+ 0.02
 *   Circle (B)          copy the current LUT values for this distance (a starting point)
 *   Cross (A)           RECORD this distance + velocity + hood
 *   Square (X)          delete the last recorded point
 *
 * STAND STILL while shooting here (shoot-on-the-move changes the aim when moving).
 * Recorded points are printed sorted, as Java arrays ready to paste into Shooter.java
 * (also written to the robot log under the tag "LUT").
 */
@TeleOp(name = "TUNE: LUT Finder (start 72,72)", group = "Tuning")
public class LutFinder extends MainTeleOp {

    public static double START_X = 72, START_Y = 72, START_HEADING_DEG = 90;

    private double vel = 1300;
    private double hood = Shooter.HOOD_POS;
    private final List<double[]> points = new ArrayList<>(); // {distance, velocity, hood}

    private boolean lastUp, lastDown, lastLeft, lastRight, lastA, lastB, lastX;

    @Override protected boolean useSavedPose()   { return false; }
    @Override protected boolean savePoseOnStop() { return false; }
    @Override protected String title()           { return "LUT FINDER (start 72, 72, 90°)"; }

    @Override
    protected Pose startPose() {
        return new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
    }

    @Override
    protected void extraControls(Pose pose) {
        if (gamepad1.dpad_up && !lastUp)       vel += 25;
        if (gamepad1.dpad_down && !lastDown)   vel = Math.max(0, vel - 25);
        if (gamepad1.dpad_right && !lastRight) hood = Math.min(1, hood + 0.02);
        if (gamepad1.dpad_left && !lastLeft)   hood = Math.max(0, hood - 0.02);

        if (gamepad1.circle && !lastB) {
            double d = shooter.distance();
            vel  = Shooter.lut(Shooter.LUT_DIST, Shooter.LUT_VEL, d);
            hood = Shooter.lut(Shooter.LUT_DIST, Shooter.LUT_HOOD, d);
        }
        if (gamepad1.cross && !lastA) {
            double[] p = { shooter.distance(), vel, hood };
            points.add(p);
            RobotLog.ii("LUT", "point: dist=%.1f vel=%.0f hood=%.2f", p[0], p[1], p[2]);
            RobotLog.ii("LUT", arrays());
        }
        if (gamepad1.square && !lastX && !points.isEmpty()) points.remove(points.size() - 1);

        lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;
        lastA = gamepad1.cross; lastB = gamepad1.circle; lastX = gamepad1.square;

        shooter.setOverride(vel, hood);
    }

    @Override
    protected void extraTelemetry() {
        telemetry.addLine();
        telemetry.addLine("── LUT FINDER ──  (stand still when shooting)");
        telemetry.addData("Distance", "%.1f in", shooter.distance());
        telemetry.addData("Manual velocity", "%.0f   (DPAD U/D)", vel);
        telemetry.addData("Manual hood", "%.2f   (DPAD L/R)", hood);
        telemetry.addLine("A record | X undo | B load LUT value | LB shoot");
        telemetry.addLine();
        telemetry.addData("Recorded", points.size());
        for (double[] p : sorted()) {
            telemetry.addLine(String.format(Locale.US, "  %5.1f in   vel %4.0f   hood %.2f", p[0], p[1], p[2]));
        }
        if (!points.isEmpty()) {
            telemetry.addLine();
            telemetry.addLine(arrays());
        }
    }

    private List<double[]> sorted() {
        List<double[]> s = new ArrayList<>(points);
        s.sort(Comparator.comparingDouble(p -> p[0]));
        return s;
    }

    /** The recorded points as Java arrays, ready to paste into Shooter.java. */
    private String arrays() {
        StringBuilder d = new StringBuilder("LUT_DIST = { ");
        StringBuilder v = new StringBuilder("LUT_VEL  = { ");
        StringBuilder h = new StringBuilder("LUT_HOOD = { ");
        List<double[]> s = sorted();
        for (int i = 0; i < s.size(); i++) {
            String sep = i < s.size() - 1 ? ", " : " ";
            d.append(String.format(Locale.US, "%.0f", s.get(i)[0])).append(sep);
            v.append(String.format(Locale.US, "%.0f", s.get(i)[1])).append(sep);
            h.append(String.format(Locale.US, "%.2f", s.get(i)[2])).append(sep);
        }
        return d.append("};\n").append(v).append("};\n").append(h).append("};").toString();
    }
}
