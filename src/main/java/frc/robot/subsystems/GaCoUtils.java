package frc.robot.subsystems;

public final class GaCoUtils {

    /**
     * Don't let anyone instantiate this class.
     */
    public GaCoUtils() {}

    public static double clip(double input, double min, double max) {
        return Math.max(min, Math.min(input, max));
    }
}
