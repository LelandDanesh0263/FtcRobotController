package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.VisionObservation.Alliance;

/** BIOBUZZ manual TU10, section 9.9. No field coordinates or scoring readiness are inferred. */
public final class BioBuzzTargets {
    private BioBuzzTargets() {}

    public enum Cell {
        RED_OPPOSITE_AUDIENCE("red/opposite-audience", Alliance.RED, 30, "RED SCORING"),
        RED_AUDIENCE("red/audience", Alliance.RED, 34, "RED AUDIENCE"),
        BLUE_AUDIENCE("blue/audience", Alliance.BLUE, 38, "BLUE AUDIENCE"),
        BLUE_OPPOSITE_AUDIENCE("blue/opposite-audience", Alliance.BLUE, 42, "BLUE SCORING");

        public final String targetId;
        public final Alliance alliance;
        public final int firstTagId;
        public final String sdkClusterName;

        Cell(String targetId, Alliance alliance, int firstTagId, String sdkClusterName) {
            this.targetId = targetId;
            this.alliance = alliance;
            this.firstTagId = firstTagId;
            this.sdkClusterName = sdkClusterName;
        }

        public int[] tagIds() {
            return new int[] {firstTagId, firstTagId + 1, firstTagId + 2, firstTagId + 3};
        }
    }

    public static Cell forTagId(int tagId) {
        for (Cell cell : Cell.values()) {
            if (tagId >= cell.firstTagId && tagId < cell.firstTagId + 4) return cell;
        }
        return null;
    }

    /** Names match the BIOBUZZ cluster metadata bundled with FTC SDK 12.0.0. */
    public static Cell forSdkClusterName(String name) {
        for (Cell cell : Cell.values()) {
            if (cell.sdkClusterName.equals(name)) return cell;
        }
        return null;
    }
}
