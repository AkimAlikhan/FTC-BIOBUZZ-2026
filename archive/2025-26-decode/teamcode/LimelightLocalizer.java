package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * ─────────────────────────────────────────────────────────────────────────────
 *  LIMELIGHT LOCALIZER  —  MegaTag2 absolute field position for DECODE season
 * ─────────────────────────────────────────────────────────────────────────────
 *
 *  HOW IT WORKS
 *  ─────────────
 *  MegaTag2 fuses the robot's IMU heading (fed from Pedro Pathing each loop)
 *  with Limelight's AprilTag detections to produce a stable, ambiguity-free
 *  x/y position on the field. Heading itself is NOT taken from Limelight —
 *  Pedro's odometry heading remains authoritative.
 *
 *  ONE-TIME LIMELIGHT DASHBOARD SETUP  (192.168.43.42:5801)
 *  ──────────────────────────────────────────────────────────
 *  1. Open the AprilTag pipeline → "Advanced" tab → enable "Full 3D".
 *  2. In the "Robot Space" section, enter the camera mount position relative
 *     to the CENTER of your robot's footprint:
 *       Forward offset  (X, inches)  — positive = camera in front of center
 *       Left   offset  (Y, inches)  — positive = camera to the left
 *       Height         (Z, inches)  — height above the floor
 *       Pitch (degrees)             — UPWARD TILT = alpha from your sketch
 *                                     e.g. if camera tilts 10° up, enter 10
 *       Roll / Yaw = 0 unless the camera is physically rotated.
 *  3. Save & click "Reboot Pipeline".
 *  The Limelight compensates for the tilt internally — no math needed here.
 *
 *  COORDINATE CONVERSION
 *  ──────────────────────
 *  Limelight uses the FTC-standard field frame:
 *      origin (0, 0) = centre of the 144 × 144 inch field
 *      X positive    = toward audience wall (or one specific wall — see below)
 *      Y positive    = perpendicular, toward the other wall
 *
 *  Pedro Pathing uses a corner-origin frame (0 → 144 on both axes).
 *  Conversion assuming same axis orientation:
 *      pedroX = limelightX + 72
 *      pedroY = limelightY + 72
 *
 *  ⚠  AXIS ORIENTATION NOTE
 *  If the reported position is mirrored or rotated versus reality, adjust
 *  the two conversion lines in update() — e.g. flip one axis:
 *      pedroX = -limelightX + 72   // flip X
 *  Verify by driving to a known field position and checking telemetry.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 */
public class LimelightLocalizer {

    // ── Tunables ───────────────────────────────────────────────────────────────

    /** Hardware-map name for the Limelight — must match your robot configuration. */
    private static final String LIMELIGHT_NAME   = "limelight";

    /** AprilTag pipeline index set up in the Limelight web interface. */
    private static final int    APRILTAG_PIPELINE = 0;

    /** Poll rate in Hz. 100 is the recommended maximum for FTC. */
    private static final int    POLL_RATE_HZ      = 100;

    /**
     * Maximum age (ms) of the Limelight result we will trust.
     * Raise if you see frequent rejections; lower for tighter freshness.
     */
    private static final long   MAX_STALENESS_MS  = 75;

    /**
     * Minimum number of AprilTags visible before we trust the fix.
     * 1 works fine with MegaTag2. Raise to 2 for extra robustness.
     */
    private static final int    MIN_TAG_COUNT     = 1;

    /**
     * Minimum tag count required to also correct the robot's heading from
     * MegaTag1 (botpose). MegaTag1 heading is computed geometrically from tags,
     * so 2+ tags gives a reliable value. With 1 tag it can be noisy/ambiguous.
     * Set to 99 to disable heading correction entirely.
     */
    private static final int    MIN_TAGS_FOR_HEADING = 1;

    // ── Internal state ─────────────────────────────────────────────────────────
    private Limelight3A limelight;  // null if not present in hardware map

    private boolean lastFixValid   = false;
    private double  lastFixHeading = 0.0;  // last corrected heading (radians)
    private int     fixCount       = 0;    // total accepted fixes this session

    // ── Debug info (populated every update, read via getDebugInfo()) ───────────
    private String debugInfo = "no data yet";

    // ── Constructor ────────────────────────────────────────────────────────────

    /**
     * Initialises and starts the Limelight. Call once in OpMode.init().
     */
    public LimelightLocalizer(HardwareMap hardwareMap) {
        Limelight3A ll = null;
        try {
            ll = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
            ll.pipelineSwitch(APRILTAG_PIPELINE);
            ll.setPollRateHz(POLL_RATE_HZ);
            // Not started here — call start() explicitly when needed
        } catch (Exception e) {
            // Limelight not present in hardware config — localization disabled
            ll = null;
        }
        limelight = ll;
    }

    /** Turns the Limelight on. Call before update() during a relocalization sequence. */
    public void start() {
        if (limelight != null) limelight.start();
    }

    // ── Main update method ─────────────────────────────────────────────────────

    /**
     * Call once per loop iteration (before reading the pose for control logic).
     *
     * <p>Feeds the current robot heading to MegaTag2, checks for a valid
     * AprilTag fix, and — if the fix passes all quality checks — corrects
     * <em>only the heading</em> in the Pedro follower.  X/Y position from
     * Pedro odometry is always preserved.</p>
     *
     * @param follower  Pedro Pathing Follower (heading is read; pose may be written)
     * @return          {@code true} if a valid fix was applied this cycle
     */
    public boolean update(Follower follower) {

        // ── 0. Skip entirely if Limelight wasn't found at init ───────────────
        if (limelight == null) return false;

        // ── 1. Feed heading to MegaTag2 (Pedro radians → degrees) ────────────
        double headingDeg = Math.toDegrees(follower.getPose().getHeading());
        limelight.updateRobotOrientation(headingDeg);

        // ── 2. Fetch latest result ────────────────────────────────────────────
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            lastFixValid = false;
            return false;
        }

        // ── 3. Get MegaTag2 bot-pose (fused with IMU heading) ─────────────────
        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) {
            lastFixValid = false;
            return false;
        }

        // ── 4. Reject stale data ──────────────────────────────────────────────
        if (result.getStaleness() > MAX_STALENESS_MS) {
            lastFixValid = false;
            return false;
        }

        // ── 5. Reject if too few tags visible ─────────────────────────────────
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        int tagCount = (tags != null) ? tags.size() : 0;

        // Build debug string — tag IDs + raw Limelight position (before conversion)
        StringBuilder sb = new StringBuilder();
        sb.append("tags seen: ").append(tagCount);
        if (tags != null) {
            sb.append(" [");
            for (LLResultTypes.FiducialResult t : tags)
                sb.append("id").append(t.getFiducialId()).append(" ");
            sb.append("]");
        }
        Pose3D rawPose = result.getBotpose_MT2();
        if (rawPose != null) {
            sb.append(String.format("  raw:(%.1f, %.1f)",
                    rawPose.getPosition().x, rawPose.getPosition().y));
        }
        if (tagCount >= MIN_TAGS_FOR_HEADING) {
            Pose3D mt1 = result.getBotpose();
            if (mt1 != null) sb.append(String.format("  hdg:%.1f°", mt1.getOrientation().getYaw()));
        }
        sb.append("  staleness:").append(result.getStaleness()).append("ms");
        debugInfo = sb.toString();

        if (tagCount < MIN_TAG_COUNT) {
            lastFixValid = false;
            return false;
        }

        // ── 6. Require a heading reading from MegaTag1 ───────────────────────
        //      MegaTag2 heading = what we fed in (circular) — useless for correction.
        //      MegaTag1 (getBotpose()) computes heading geometrically from tags.
        if (tagCount < MIN_TAGS_FOR_HEADING) {
            lastFixValid = false;
            return false;
        }
        Pose3D botposeMT1 = result.getBotpose();
        if (botposeMT1 == null) {
            lastFixValid = false;
            return false;
        }

        // ── 7. Convert Limelight yaw → Pedro heading ──────────────────────────
        //      Limelight yaw is mirrored vs Pedro's convention — apply 180-yaw.
        //      (e.g. LL reports 135° where Pedro expects 45°)
        double llYawDeg   = botposeMT1.getOrientation().getYaw();
        double newHeading = Math.toRadians(180.0 - llYawDeg);

        // ── 8. Apply heading only — x/y stay at Pedro's current odometry ─────
        Pose current = follower.getPose();
        follower.setPose(new Pose(current.getX(), current.getY(), newHeading));
        lastFixValid   = true;
        lastFixHeading = newHeading;
        fixCount++;
        return true;
    }

    // ── Accessors (for telemetry) ───────────────────────────────────────────────

    /** Heading (radians, Pedro convention) of the last accepted fix. */
    public double getLastHeading() { return lastFixHeading; }

    /** Heading (degrees) of the last accepted fix — convenience for telemetry. */
    public double getLastHeadingDeg() { return Math.toDegrees(lastFixHeading); }

    /** Whether the most recent update() call produced and applied a valid fix. */
    public boolean isLastFixValid() { return lastFixValid; }

    /** Total number of accepted fixes applied since init. */
    public int getFixCount() { return fixCount; }

    /**
     * Raw debug string — tag IDs seen, raw Limelight x/y (field-centre coords),
     * and data staleness. Use this to verify the DECODE field map is loaded.
     *
     * DECODE goal tag IDs are 11–16. If you see other IDs, the wrong field map
     * is loaded and you need to update Limelight firmware or upload the DECODE map.
     * Raw x/y should stay within ±72 inches (i.e. pedro coords 0–144 after +72).
     */
    public String getDebugInfo() { return debugInfo; }

    // ── Lifecycle ──────────────────────────────────────────────────────────────

    /** Call in OpMode.stop() to shut down the Limelight cleanly. */
    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
