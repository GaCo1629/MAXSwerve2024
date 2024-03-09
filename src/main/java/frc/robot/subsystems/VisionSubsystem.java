package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.Globals;
import frc.robot.utils.Limelight;
import frc.robot.utils.Target;

public class VisionSubsystem extends SubsystemBase{

    double  lastNoteTargetHash ;
    boolean needFreshNote;

    public VisionSubsystem (){
        lastNoteTargetHash = 0;
        needFreshNote = false;
    }

    @Override
    public void periodic(){
        getSpeakerTargetFromAprilTag();
        getNoteTarget();

        SmartDashboard.putString("Speaker", Globals.speakerTarget.toString());
        SmartDashboard.putString("Note"   , Globals.noteTarget.toString());

        SmartDashboard.putBoolean("ValidSpeaker", Globals.speakerTarget.valid);
        SmartDashboard.putBoolean("ValidNote", Globals.noteTarget.valid);
    }

    public void flushNoteTargets() {
        Globals.noteTarget  = new Target();
        needFreshNote        = true;
    }

    //  ======================  Speaker Tracking Vision processing
    public Target getSpeakerTargetFromAprilTag() {
        double y = 0;
        double range = 0;
        double bearing = 0;
        Target aTarget;

        if (Limelight.getTV("limelight")) {

            bearing = Limelight.getTX("limelight");
            y = Limelight.getTY("limelight");

            range   = (VisionConstants.speakerTagHeightAboveCamera / Math.tan(Math.toRadians(VisionConstants.speakerCameraAngle + y))) + VisionConstants.speakerCameraCenterOffset;
            range *= VisionConstants.rangeAdjustV2O ; 
            aTarget = new Target(true, range, bearing);
        }else{
            aTarget = new Target();
        }
        Globals.speakerTarget = aTarget;
        return aTarget;

    }

    //  ======================  Note Tracking Vision processing
    public Target getNoteTarget() {
        double x = 0;
        double y = 0;
        double a = 0;
        double range = 0;
        double hash  = 0;
        Target aTarget = new Target();

        if (Limelight.getTV("limelight-note")) {

            x = Limelight.getTX("limelight-note");
            y = Limelight.getTY("limelight-note");
            a = Limelight.getTA("limelight-note");
            hash = x + y + a;  // come up with a value that will cprobably change for each note target.

            // do we need a guarenteed fresh Note?
            if ((hash != lastNoteTargetHash) || !needFreshNote) {
                range = (Math.tan(Math.toRadians(VisionConstants.noteCameraAngle + y)) * VisionConstants.noteCameraHeight) + VisionConstants.noteRollerOffset;
                aTarget = new Target(true, range, x);
                lastNoteTargetHash = hash ;
                needFreshNote = false;
            }
        }

        Globals.noteTarget = aTarget;
        return aTarget;
    }
}

