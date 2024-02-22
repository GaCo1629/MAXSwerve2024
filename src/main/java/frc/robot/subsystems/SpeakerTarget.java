package frc.robot.subsystems;

public class SpeakerTarget {
    
    public boolean valid;
    public double range;
    public double bearing;
    public double elevation;

    public SpeakerTarget(boolean valid, double range, double bearing, double elevation) {
        this.valid = valid;
        this.range = range;
        this.bearing = bearing;
        this.elevation = elevation;
    }

    public SpeakerTarget() {
        this.valid = false;
        this.range = 0;
        this.bearing = 0;
        this.elevation = 0;
    }

    public String toString() {
        return String.format("V R:B:E %s %5.2f %5.1f Deg, %5.1f Deg", valid, range, bearing, elevation);
    }
}
