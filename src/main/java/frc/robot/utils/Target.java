package frc.robot.utils;

public class Target {
    
    public boolean valid;
    public double range;
    public double bearingDeg;

    public Target(boolean valid, double range, double bearing) {
        this.valid = valid;
        this.range = range;
        this.bearingDeg = bearing;
    }

    public Target() {
        this.valid = false;
        this.range = 0;
        this.bearingDeg = 0;
    }

    public String toString() {
        return String.format("V R:B %s %5.2f %5.1f Deg", valid, range, bearingDeg);
    }
}
