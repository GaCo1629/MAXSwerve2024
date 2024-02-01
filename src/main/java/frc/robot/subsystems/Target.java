package frc.robot.subsystems;

public class Target {
    
    public boolean valid;
    public double range;
    public double bearing;
    public double elevation;

    public Target(boolean valid, double range, double bearing, double elevation) {
        this.valid = valid;
        this.range = range;
        this.bearing = bearing;
        this.elevation = elevation;
    }

    public Target() {
        this.valid = false;
        this.range = 0;
        this.bearing = 0;
        this.elevation = 0;
    }
}
