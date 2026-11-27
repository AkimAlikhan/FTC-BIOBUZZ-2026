package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;

import java.util.Map;

/**
 * Field helpers shared by Auto and TeleOp.
 *
 * Blue = red rotated 180° around the field center (72, 72):
 *   x -> 144 - x,  y -> 144 - y,  heading -> heading + 180°
 *
 * Auto -> TeleOp handoff uses the SDK blackboard (same keys as 2025-26),
 * which survives between OpModes until the Robot Controller app restarts.
 */
public final class Field {
    public static final String KEY_X = "lastX", KEY_Y = "lastY", KEY_HEADING = "lastHeading";
    public static final String KEY_RED = "isRed";

    private Field() {}

    /** A red-side pose (inches, degrees), mirrored for blue. */
    public static Pose pose(boolean red, double x, double y, double headingDeg) {
        double h = Math.toRadians(headingDeg);
        return red ? new Pose(x, y, h) : new Pose(144 - x, 144 - y, h + Math.PI);
    }

    /** A red-side point (no heading), mirrored for blue. */
    public static Pose point(boolean red, double x, double y) {
        return red ? new Pose(x, y) : new Pose(144 - x, 144 - y);
    }

    /** A red-side heading in degrees, mirrored for blue, in radians. */
    public static double heading(boolean red, double headingDeg) {
        return Math.toRadians(headingDeg) + (red ? 0 : Math.PI);
    }

    public static void savePose(Map<String, Object> blackboard, Pose pose, boolean red) {
        blackboard.put(KEY_X, pose.x());
        blackboard.put(KEY_Y, pose.y());
        blackboard.put(KEY_HEADING, pose.heading());
        blackboard.put(KEY_RED, red);
    }

    /** The pose saved by the last Auto (or TeleOp), or null if there is none. */
    public static Pose loadPose(Map<String, Object> blackboard) {
        Object x = blackboard.get(KEY_X), y = blackboard.get(KEY_Y), h = blackboard.get(KEY_HEADING);
        if (!(x instanceof Double) || !(y instanceof Double) || !(h instanceof Double)) return null;
        return new Pose((Double) x, (Double) y, (Double) h);
    }

    /** Alliance saved by the last Auto, or the given default. */
    public static boolean loadRed(Map<String, Object> blackboard, boolean fallback) {
        Object r = blackboard.get(KEY_RED);
        return r instanceof Boolean ? (Boolean) r : fallback;
    }
}
