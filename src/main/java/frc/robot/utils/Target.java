package frc.robot.utils;

public class Target {
    
    public boolean valid;
    public double range;
    public double bearingDeg;
    public double bearingRad;

    public Target(boolean valid, double range, double bearingDeg) {
        this.valid = valid;
        this.range = range;
        this.bearingDeg = bearingDeg;
        this.bearingRad = Math.toRadians(bearingDeg);
    }

    public Target() {
        this.valid = false;
        this.range = 0;
        this.bearingDeg = 0;
        this.bearingRad = 0;
    }

    public String toString() {
        return String.format("V R:B %s %5.2f %5.1f Deg", valid, range, bearingDeg);
    }
}
