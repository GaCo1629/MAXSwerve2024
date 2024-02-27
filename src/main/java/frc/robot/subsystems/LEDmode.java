package frc.robot.subsystems;

public enum LEDmode {
    NONE,
    ALLIANCE,           // Display Alliance color
    RAINBOW,            // Show a pretty Rainbow
    NOTE_COLLECTING,         // Seeking a Note to collect
    NOTE_DETECTED,      // Note is visible
    NOTE_HOLDING,       // Note is in robot
    SEEKING,            // Seeking a speaker to score
    SPEAKER_DETECTED,   // Speaker Apriltag has been detected
    SHOOTING,           // In the process of preparing to shoot and waiting to shoot 
    SPEEDOMETER,        // Displaying robot speed on power meter.
    SYSTEM_ERROR,       // Displaying system error code
    DEFAULT
}
