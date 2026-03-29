package frc.robot.lib.leds;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * Lightweight section cache for LED ranges. Call addSection(...) once at init
 * and then getSectionStart/getSectionEnd/getSectionLength are cheap primitive reads.
 */
public final class LedSectionConfig {
    // map from section name -> int[]{start, end, length}
    private static final Map<String, int[]> sections = new HashMap<>();

    private LedSectionConfig() {}

    /**
     * Register a section. Call once during initialization (e.g., in robot constructor).
     */
    public static synchronized void addSection(String name, int start, int end) {
        if (name == null) {
            return;
        }
        if (end < start) {
            // swap to ensure start <= end
            int t = start;
            start = end;
            end = t;
        }
        int length = Math.max(0, end - start);
        sections.put(name, new int[] { start, end, length });
    }

    /** Returns the start index for the named section; returns 0 if unknown. */
    public static int getSectionStart(String name) {
        int[] v = sections.get(name);
        return v != null ? v[0] : 0;
    }

    /** Returns the end index (exclusive) for the named section; returns 0 if unknown. */
    public static int getSectionEnd(String name) {
        int[] v = sections.get(name);
        return v != null ? v[1] : 0;
    }

    /** Returns the cached length (end - start) for the named section; 0 if unknown. */
    public static int getSectionLength(String name) {
        int[] v = sections.get(name);
        return v != null ? v[2] : 0;
    }

    /** Optionally expose a read-only view if callers need it for diagnostics. */
    public static Map<String, int[]> getAllSections() {
        return Collections.unmodifiableMap(sections);
    }
}
