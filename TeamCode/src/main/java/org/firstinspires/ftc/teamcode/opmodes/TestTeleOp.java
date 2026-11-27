package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Same as "TeleOp", but always starts at (24, 24, 90°):
 *   - ignores the pose / alliance saved by Auto,
 *   - doesn't save its own pose at the end (so it never affects the real TeleOp),
 *   - zeroes the turret at init (turret must face forward),
 *   - Triangle resets to (24, 24, 90°).
 * Alliance is still chosen in init with DPAD LEFT (red) / RIGHT (blue).
 */
@TeleOp(name = "TEST: TeleOp (start 24,24)", group = "Test")
public class TestTeleOp extends MainTeleOp {

    public static double TEST_X = 24, TEST_Y = 24, TEST_HEADING_DEG = 90;

    @Override protected boolean useSavedPose()   { return false; }
    @Override protected boolean savePoseOnStop() { return false; }
    @Override protected String title()           { return "TEST TeleOp (start 24, 24, 90°)"; }

    @Override
    protected Pose startPose() {
        return new Pose(TEST_X, TEST_Y, Math.toRadians(TEST_HEADING_DEG));
    }

    @Override
    protected void extraTelemetry() {
        Pose p = localizer.pose();
        Pose s = startPose();
        Velocity v = localizer.velocity();
        double headingDeg = Math.toDegrees(p.heading());
        double turnedDeg = Math.toDegrees(Math.atan2(Math.sin(p.heading() - s.heading()), Math.cos(p.heading() - s.heading())));

        telemetry.addLine();
        telemetry.addLine("── POSITION ──");
        telemetry.addData("X", "%.2f in", p.x());
        telemetry.addData("Y", "%.2f in", p.y());
        telemetry.addData("Heading", "%.1f°", headingDeg);
        telemetry.addData("Moved from start", "dx %+.2f  dy %+.2f  (%.2f in)  turned %+.1f°",
                p.x() - s.x(), p.y() - s.y(), Math.hypot(p.x() - s.x(), p.y() - s.y()), turnedDeg);
        telemetry.addData("Speed", "%.1f in/s  (vx %.1f  vy %.1f)  turn %.0f°/s",
                Math.hypot(v.vx, v.vy), v.vx, v.vy, Math.toDegrees(v.omega));
        telemetry.addLine("Triangle = reset to start");
    }
}
