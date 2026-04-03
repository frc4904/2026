package org.usfirst.frc4904.robot.vision;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.usfirst.frc4904.standard.util.Util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

/** Manages Google tags */
public final class TagManager {
    private TagManager() {}

    // forward, left, up - z components don't matter (at least for now)
    private static Transform3d[] CAMERA_OFFSETS = {
        Util.transform3d(Units.inchesToMeters(12.5), 0, 0, 0),
        Util.transform3d(Units.inchesToMeters(-13), 0, 0, Rotation2d.k180deg.getRadians())
    };

    public record Tag(int id, Transform3d pos, Pose3d fieldPos, double time, int camera) {}

    private static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private static final ObjectMapper mapper = new ObjectMapper();
    private static final NetworkTableEntry tagsEntry, timeEntry;
    static {
        var table = NetworkTableInstance.getDefault();
        tagsEntry = table.getEntry("/dauntless/tags");
        timeEntry = table.getEntry("/dauntless/time");
    }

    private static double lastTime;
    private static List<Tag> lastTags;

    public static List<Tag> getTags() {
        double timeSeconds = timeEntry.getDouble(0);

        if (lastTime == timeSeconds && lastTags != null) return lastTags;
        lastTime = timeSeconds;

        List<Tag> tags = lastTags = new ArrayList<>();

        String json = tagsEntry.getString("");
        if (json.isEmpty()) return tags;

        try {
            JsonNode root = mapper.readTree(json);

            for (JsonNode el : root) {
                JsonNode idPath = el.path("id");
                if (idPath.isNull()) continue;
                int id = idPath.asInt();

                int camera = el.path("camera").asInt();
                double[] pos = mapper.treeToValue(el.path("pos"), double[].class);

                Optional<Pose3d> tagPose = field.getTagPose(id);
                if (tagPose.isEmpty()) {
                    System.err.println("Tag id " + id + " does not exist on field layout");
                    continue;
                }

                Transform3d robotToTag = Util.transform3d(pos[2], -pos[0], pos[1], el.path("rot").asDouble());
                Transform3d cameraOffset = CAMERA_OFFSETS[camera];

                double time = Util.epochSecondsToFPGATimestamp(el.path("time").asDouble());

                tags.add(new Tag(
                    id,
                    cameraOffset.plus(robotToTag),
                    tagPose.get(),
                    time,
                    0
                ));
            }
        } catch (JsonProcessingException e) {
            System.out.println("google tag manager parsing error!!!");
        }

        return tags;
    }

    /**
     * For usage with {@link #getTagsSince(double)}
     */
    public static double getLastTime() {
        return lastTime;
    }

    /**
     * Returns the result of {@link #getTags()} if the result has changed since {@code time}, otherwise an empty list.
     * The time is in epoch seconds, the same units as {@link #getLastTime()}
     */
    public static List<Tag> getTagsSince(double time) {
        List<Tag> tags = getTags();
        return time < lastTime ? tags : Collections.emptyList();
    }

}
