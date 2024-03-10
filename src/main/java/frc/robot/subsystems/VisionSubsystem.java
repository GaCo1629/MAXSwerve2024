package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.BackImageSource;
import frc.robot.utils.FrontImageSource;
import frc.robot.utils.Globals;
import frc.robot.utils.Limelight;
import frc.robot.utils.Target;

public class VisionSubsystem extends SubsystemBase{

    double  lastNoteTargetHash ;
    boolean needFreshNote;

    public VisionSubsystem (){
        lastNoteTargetHash = 0;
        needFreshNote = false;
        setFrontImageSource(FrontImageSource.NOTE);
        setBackImageSource(BackImageSource.SPEAKER);
    }

    @Override
    public void periodic(){
        getSpeakerTarget();
        if (Globals.frontSource == FrontImageSource.NOTE) {
            getNoteTarget();
            Globals.ampTarget = new Target();
        } else {
            getAmpTarget();
            Globals.noteTarget = new Target();
        }

        SmartDashboard.putString("Speaker", Globals.speakerTarget.toString());
        SmartDashboard.putString("Note"   , Globals.noteTarget.toString());
        SmartDashboard.putString("Amp"   ,  Globals.ampTarget.toString());

        SmartDashboard.putBoolean("ValidSpeaker", Globals.speakerTarget.valid);
        SmartDashboard.putBoolean("ValidNote", Globals.noteTarget.valid);
        SmartDashboard.putBoolean("ValidAmp", Globals.ampTarget.valid);
    }

    public void flushNoteTargets() {
        Globals.noteTarget  = new Target();
        needFreshNote        = true;
    }

    public static void setFrontImageSource(FrontImageSource source) {
        // Use the network tables to update the camera pipeline
        if (source != Globals.frontSource) {
            if (source == FrontImageSource.NOTE) {
                Limelight.setPipelineIndex("limelight-note", 0);
            } else {
            Limelight.setPipelineIndex("limelight-note", 1);
                if (DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == Alliance.Red)) {
                    Limelight.setTagPriority("limelight-note", 5);
                } else {
                    Limelight.setTagPriority("limelight-note", 6);
                }
            }
        }
        Globals.frontSource = source;
    }

    public static void setBackImageSource(BackImageSource source) {
        // Use the network tables to update the camera pipeline
        if (source != Globals.backSource) {
            if (source == BackImageSource.SPEAKER) {
                if (DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == Alliance.Red)) {
                    Limelight.setTagPriority("limelight", 4);
                } else {
                    Limelight.setTagPriority("limelight", 7);
                }
            } else {
                Limelight.setTagPriority("limelight", -1);
            }
        }
        Globals.backSource = source;
    }
    
    //  ======================  Speaker Tracking Vision processing
    public Target getSpeakerTarget() {
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
            hash = x + y + a;  // come up with a value that will probably change for each note target.

            // do we need a guarenteed fresh Note?
            if ((hash != lastNoteTargetHash) || !needFreshNote) {
                if (a > VisionConstants.noteAreaThreshold){
                    range = (Math.tan(Math.toRadians(VisionConstants.noteCameraAngle + y)) * VisionConstants.noteCameraHeight) + VisionConstants.noteRollerOffset;
                    aTarget = new Target(true, range, x);
                    lastNoteTargetHash = hash ;
                    needFreshNote = false;
                }
                
            }
        }

        Globals.noteTarget = aTarget;
        return aTarget;
    }

    //  ======================  AMP Tracking Vision processing
    public Target getAmpTarget() {
        double x = 0;
        double y = 0;
        double a = 0;
        double range = 0;
        Target aTarget = new Target();

        // Use the X value for Bearing, but use the area for range.
        // It would be nice to use the Y value for range, but the baton tilt will not be stable, so just use the area with a minimum 

        if (Limelight.getTV("limelight-note")) {

            x = Limelight.getTX("limelight-note");
            y = Limelight.getTY("limelight-note");
            a = Limelight.getTA("limelight-note");

            if (a > VisionConstants.ampAreaThreshold){
                range = (Math.tan(Math.toRadians(VisionConstants.noteCameraAngle + y)) * VisionConstants.noteCameraHeight) + VisionConstants.noteRollerOffset;
                aTarget = new Target(true, range, x);
            }
        }
        Globals.noteTarget = aTarget;
        return aTarget;
    }
}

