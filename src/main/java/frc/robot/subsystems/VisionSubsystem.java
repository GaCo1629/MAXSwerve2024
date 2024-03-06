package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.Globals;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Target;

public class VisionSubsystem extends SubsystemBase{

    public VisionSubsystem (){

    }

    @Override
    public void periodic(){
        getSpeakerTarget();
        getNoteTarget();

        SmartDashboard.putString("Speaker", Globals.speakerTarget.toString());
        SmartDashboard.putString("Note"   , Globals.noteTarget.toString());

        SmartDashboard.putBoolean("ValidSpeaker", Globals.speakerTarget.valid);
        SmartDashboard.putBoolean("ValidNote", Globals.noteTarget.valid);
    }

    //  ======================  Speaker Tracking Vision processing
    public Target getSpeakerTarget() {
        double y = 0;
        double range = 0;
        double bearing = 0;
        Target aTarget;

        if (LimelightHelpers.getTV("limelight")) {

            bearing = LimelightHelpers.getTX("limelight");
            y = LimelightHelpers.getTY("limelight");

            range   = VisionConstants.speakerTagHeightAboveCamera / Math.tan(Math.toRadians(VisionConstants.speakerCameraAngle + y));
            aTarget = new Target(true, range, bearing);
        }else{
            aTarget = new Target();
        }
        Globals.noteTarget = aTarget;
        return aTarget;

    }

    //  ======================  Note Tracking Vision processing
    public Target getNoteTarget() {
        double y = 0;
        double range = 0;
        double bearing = 0;
        Target aTarget;

        if (LimelightHelpers.getTV("limelight-note")) {

            bearing = LimelightHelpers.getTX("limelight-note");
            y       = LimelightHelpers.getTY("limelight-note");
            range = Math.tan((Math.toRadians(VisionConstants.noteCameraAngle + y) * VisionConstants.noteCameraHeight) + VisionConstants.noteRollerOffset);

            aTarget = new Target(true, range, bearing);
        }else{
            aTarget = new Target();
        }
        Globals.noteTarget = aTarget;
        return aTarget;
    }
}

