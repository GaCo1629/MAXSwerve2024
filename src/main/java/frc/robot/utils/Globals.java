package frc.robot.utils;

import frc.robot.Constants.ShooterConstants;

public final class Globals{
    public static boolean enableLIftSubsystem   = true;  // set to false if climber removed.

    public static boolean gyroHasBeenReset      = false;
    public static boolean batonIsDown           = false;
    public static boolean robotAtHeading        = false;
    public static boolean startNoteFinding      = false;
    public static boolean noteInIntake          = false;
    public static boolean noteAtShooter         = false;
    public static boolean liftIsEnabled         = false;
    public static boolean defenseActive         = false;

    public static FrontImageSource frontSource;
    public static BackImageSource  backSource;

    public static BatonState batonState = BatonState.IDLE;

    public static double  lastShooterSpeed      = ShooterConstants.baseShooterSpeed;
    public static double  speed                 = 0;
    public static boolean startingLocationSet   = false;

    public static Target  speakerTarget         = new Target();
    public static Target  noteTarget            = new Target();
    public static Target  odoTarget             = new Target();
    public static Target  ampTarget             = new Target();
    public static Target  lobTarget             = new Target();
 
    private static LEDmode ledMode               = LEDmode.ALLIANCE;

    private static boolean amplifyingEnabled     = false;
    private static boolean noteTrackingEnabled   = false;
    private static boolean speakerTrackingEnabled= false;

    public  static int     driveSubsystemFaults = 0;
    public  static int     batonSubsystemFaults = 0;
    public  static int     liftSubsystemFaults  = 0;
    

    public static void setNoteTracking(boolean on) {
        noteTrackingEnabled = on;
    }

    public static boolean getNoteTracking() {
        return noteTrackingEnabled;
    }

    public static void setStartNoteFinding() {
        startNoteFinding = true;
    }

    public static void setDefenseMode(boolean on) {
        defenseActive = on;
    }

    public static void setSpeakerTracking(boolean on) {
        speakerTrackingEnabled = on;
        if(speakerTrackingEnabled){
            Limelight.setLEDMode_ForceOn("");
        } else {
            Limelight.setLEDMode_ForceOff("");
        }
    }

    public static boolean getSpeakerTracking() {
        return speakerTrackingEnabled;
    }

    public static void setAmplifying(boolean on) {
        amplifyingEnabled = on;
    }

    public static boolean getAmplifying() {
        return amplifyingEnabled;
    }

    public static void setLEDMode(LEDmode mode) {
        ledMode = mode;
    }

    public static LEDmode getLEDMode() {
        return ledMode;
    }
}
