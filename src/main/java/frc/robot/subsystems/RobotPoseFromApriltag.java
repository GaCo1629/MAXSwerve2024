package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

public class RobotPoseFromApriltag {
    
    public boolean valid;
    public Pose2d robotPose;
    public double bearing;

    public RobotPoseFromApriltag(boolean valid, Pose2d robotPose) {
        this.valid = valid;
        this.robotPose = robotPose;
    }

    public RobotPoseFromApriltag() {
        this.valid = false;
        this.robotPose = new Pose2d();
    }

    public String toString() {
        return String.format("V Pose %s %s", valid, robotPose.toString());
    }
}
