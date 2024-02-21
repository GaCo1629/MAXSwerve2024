package frc.robot.subsystems;

public class ApriltagTarget {
    
    public boolean valid;
    public double range;
    public double bearing;
    public double elevation;

    public ApriltagTarget(boolean valid, double range, double bearing, double elevation) {
        this.valid = valid;
        this.range = range;
        this.bearing = bearing;
        this.elevation = elevation;
    }

    public ApriltagTarget() {
        this.valid = false;
        this.range = 0;
        this.bearing = 0;
        this.elevation = 0;
    }

    public String toString() {
        return String.format("R:B:E %5.2f %5.1f Deg, %5.1f Deg", range, bearing, elevation);
    }
}
