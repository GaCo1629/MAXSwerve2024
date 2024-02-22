package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase{

    public VisionSubsystem (){

    }

    @Override
    public void periodic(){
        if (DriverStation.isDisabled()){
            getRobotPoseFromApriltag();
        } else {
            getSpeakerTarget();
            getNoteTarget();
        }
    }

    //  ======================  Vision processing
    public RobotPoseFromApriltag getRobotPoseFromApriltag() {

        if (LimelightHelpers.getTV("")) {
            Globals.robotPoseFromApriltag = new RobotPoseFromApriltag(true, LimelightHelpers.getBotPose2d_wpiBlue(""));
        } else {
            Globals.robotPoseFromApriltag = new RobotPoseFromApriltag();
        }

        return  Globals.robotPoseFromApriltag;
    }

    //  ======================  Vision processing
    public SpeakerTarget getSpeakerTarget() {
        double x,y,z = 0;
        double range = 0;
        double bearing = 0;
        double elevation = 0;
        SpeakerTarget aTarget;

        Pose3d targetLocation = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");

        x = targetLocation.getX();
        y = targetLocation.getZ();
        z = -targetLocation.getY();

        // SmartDashboard.putString("Target Coord", String.format("X:Y:Z %5.2f  %5.2f  %5.2f ", x,y,z));

        range = Math.hypot(x, y);
        bearing = -Math.atan2(x, y);
        elevation = Math.atan2(z, range);

        if (range > 0.5) {
            aTarget = new SpeakerTarget(true, range, Math.toDegrees(bearing), Math.toDegrees(elevation));
        }else{
            aTarget = new SpeakerTarget();
        }
        Globals.speakerTarget = aTarget;
        return aTarget;
    }

    //  ======================  Vision processing
    public NoteTarget getNoteTarget() {
        double y = 0;
        double range = 0;
        double bearing = 0;
        NoteTarget aTarget;

        if (LimelightHelpers.getTV("limelight-note")) {

            bearing = LimelightHelpers.getTX("limelight-note");
            y = LimelightHelpers.getTY("limelight-note");

            range = Math.tan((Math.toRadians(VisionConstants.noteCameraAngle + y) * VisionConstants.noteCameraHeight) + VisionConstants.noteRollerOffset);
            aTarget = new NoteTarget(true, range, bearing);
        }else{
            aTarget = new NoteTarget();
        }
        Globals.noteTarget = aTarget;
        return aTarget;
    }


}

