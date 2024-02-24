package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;

public final class Globals{
    public static boolean enableLIftSubsystem   = false;
    public static boolean gyroHasBeenReset      = false;
    public static double  robotPitch            = 0;
    public static double  robotRoll             = 0;
    public static double  lastShooterSpeed      = ShooterConstants.baseShooterSpeed;
    public static SpeakerTarget speakerTarget   = new SpeakerTarget();
    public static NoteTarget noteTarget         = new NoteTarget();
    public static RobotPoseFromApriltag robotPoseFromApriltag = new RobotPoseFromApriltag();
    public static LEDmode ledMode               = LEDmode.ALLIANCE;

    private static boolean noteTrackingEnabled   = false;
    private static boolean speakerTrackingEnabled= false;

    public static void setNoteTracking(boolean on) {
        noteTrackingEnabled = on;
    }

    public static boolean getNoteTracking() {
        return noteTrackingEnabled;
    }

    public static void setSpeakerTracking(boolean on) {
        speakerTrackingEnabled = on;
    }

    public static boolean getSpeakerTracking() {
        return speakerTrackingEnabled;
    }

    
}
