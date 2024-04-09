package frc.robot.utils;

public enum LEDmode {
    NONE,
    ALLIANCE,           // Display Alliance color
    RAINBOW,            // Show a pretty Rainbow
    NOTE_COLLECTING,         // Seeking a Note to collect
    NOTE_DETECTED,      // Note is visible
    NOTE_HOLDING,       // Note is in robot
    SEEKING,            // Seeking a speaker to score
    SPEAKER_DETECTED,   // Speaker Apriltag has been detected
    WINDING_UP,         // We want to shoot but we aren't ready yet.
    SHOOTING,           // Actually firing when ready
    SHOOTING_TIMEOUT,   // Shooting even if not ready
    SPEEDOMETER,        // Displaying robot speed on power meter.
    LOWERING,           // Baton is still returning from raised position
    SYSTEM_ERROR,       // Displaying system error code
    WAITING,            // Waiting for any number of things   
    DONE_WAITING,
    DEFAULT
}
