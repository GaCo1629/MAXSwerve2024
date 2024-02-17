package frc.robot.subsystems;

public final class Utils {

    /**
     * Don't let anyone instantiate this class.
     */
    private Utils() {}

    public static double clip(double input, double min, double max) {
        return Math.max(min, Math.min(input, max));
    }
}
