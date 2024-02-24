// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
    DriveConstants.kFrontLeftDrivingCanId,
    DriveConstants.kFrontLeftTurningCanId,
    DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
    DriveConstants.kRearLeftDrivingCanId,
    DriveConstants.kRearLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
    DriveConstants.kRearRightDrivingCanId,
    DriveConstants.kRearRightTurningCanId,
    DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final IMUInterface imu = new IMUInterface();
  
  private PS4Controller driver;
  //private Joystick copilot_1;
  //private Joystick copilot_2;
  
  private boolean headingLocked = false;
  private double  headingSetpoint = 0;
  private double speedFactor = DriveConstants.kAtleeSpeedFactor;

  private SlewRateLimiter XLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter YLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);

  private ProfiledPIDController headingLockController;
  private PIDController         trackingController;

  private Timer       trackTimer = new Timer();
  private Rotation2d  lastHeadingOverride;
  
  // Odometry class for tracking robot pose (use pose estimator for ading vision)
  SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics,
    imu.rotation2d,
    new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition(),  m_rearRight.getPosition()
    },
    new Pose2d());
 

  /** Creates a new DriveSubsystem. */
   public DriveSubsystem(PS4Controller driver, Joystick copilot_1, Joystick copilot_2) {
    this.driver = driver;
    //this.copilot_1 = copilot_1;
    //this.copilot_2 = copilot_2;

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
              4.5, // Max module speed, in m/s
              0.4, // Drive base radius in meters. Distance from robot center to furthest module.
              new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this // Reference to this subsystem to set requirements
    );

    headingLockController = new ProfiledPIDController(AutoConstants.kPHeadingLockController, 
                                                      AutoConstants.kIHeadingLockController, 
                                                      AutoConstants.kDHeadingLockController, 
                                                      AutoConstants.kHeadingLockConstraints );
    headingLockController.enableContinuousInput(-Math.PI, Math.PI);
    
    trackingController = new PIDController(AutoConstants.kPTrackingController, 
                                                      AutoConstants.kITrackingController, 
                                                      AutoConstants.kDTrackingController);
    trackingController.enableContinuousInput(-180, 180);

    // Set the method that will be used to get rotation overrides
    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

  }


  /**
   * Initialize everything needed when Teleop first starts up.
   */
  public void init() {

    if (!Globals.gyroHasBeenReset) {
      resetHeading();
    }
    lockCurrentHeading();

    Globals.setNoteTracking(false);
    Globals.setSpeakerTracking(false);
    trackTimer.start();
    lastHeadingOverride = new Rotation2d();
  }


  @Override
  public void periodic() {

    // Update the odometry in the periodic block
    imu.update();
    odometry.update(
      imu.rotation2d,
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()}
    );

    // Look for AprilTags on the Speakers to update static position when disabled.
    if (DriverStation.isDisabled() && Globals.robotPoseFromApriltag.valid) {
      Pose2d robotPose = Globals.robotPoseFromApriltag.robotPose;          
      odometry.addVisionMeasurement(robotPose, Timer.getFPGATimestamp());
    }

    // Display Estimated Position
    SmartDashboard.putString("Estimated Pos", odometry.getEstimatedPosition().toString());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
      imu.rotation2d,
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive() {
    
    double xSpeed;
    double ySpeed;
    double rotate;
    boolean fieldRelative = true;

    // Read joystick values
    xSpeed     = squareJoystick(-MathUtil.applyDeadband(driver.getLeftY(), OIConstants.kDriveDeadband)) *  speedFactor;
    ySpeed     = squareJoystick(-MathUtil.applyDeadband(driver.getLeftX(), OIConstants.kDriveDeadband)) *  speedFactor;
    rotate     = squareJoystick(-MathUtil.applyDeadband(driver.getRightX(), OIConstants.kDriveDeadband)) * DriveConstants.kAtleeTurnFactor;
 
    // smooth out the translation requests
    xSpeed = XLimiter.calculate(xSpeed);
    ySpeed = YLimiter.calculate(ySpeed);


    // TARGET TRACKING =======================================================
    if (Globals.getSpeakerTracking() && Globals.speakerTarget.valid) {
      //  TRACKING SPEAKER 
      SmartDashboard.putString("Mode", "Speaker")  ;

      // Calculate turn power to point to speaker.
      rotate = -trackingController.calculate(Globals.speakerTarget.bearing, 180);

      // Add additional rotation based on robot's sideways motion 
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        rotate += (ySpeed * 0.2);
      } else {
        rotate += (ySpeed * 0.2);   //  change sign ???        
      }
      lockCurrentHeading();  // prepare for return to heading hold

    } else if (Globals.getNoteTracking()) {
      //  TRACKING NOTE 
      SmartDashboard.putString("Mode", "Node")  ;

      if (Globals.noteTarget.valid){
        // Calculate turn power to point to note.
        rotate = trackingController.calculate(Globals.noteTarget.bearingDeg, 0) / 2.0;
        if (Math.abs(trackingController.getPositionError()) < 10){
          fieldRelative = false;
          xSpeed = Globals.noteTarget.range / 3.0;
        }
      } else {
        fieldRelative = false;
        xSpeed = 0.12;
      }
      lockCurrentHeading();  // prepare for return to heading hold

    } else {

      //  NO TRACKING
      if (rotate != 0) {
        headingLocked = false;
      } else if (!headingLocked && isNotRotating()) {
        lockCurrentHeading();
      }

      // if Heading lock is engaged, override the user input with data from PID
      if (headingLocked) {
        SmartDashboard.putString("Mode", "Auto")  ;
        // PID control.
        rotate = headingLockController.calculate(imu.headingRad, headingSetpoint);
        if (Math.abs(rotate) < 0.025) {
          rotate = 0;
        } 
      } else {
        // plain driver control
        SmartDashboard.putString("Mode", "Manual")  ;
        rotate = rotLimiter.calculate(rotate);
      }
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered    = rotate * DriveConstants.kMaxAngularSpeed;

    // Send required power to swerve drives
    driveRobot(xSpeedDelivered, ySpeedDelivered, rotDelivered, fieldRelative);

  }

  /**
   * Drives robot based on requested axis movements
   * Can be field relative or robot relative
   * 
   * @param xSpeed  Fwd speed in mps
   * @param ySpeed  Left speed in mps
   * @param rot     CCW rotation in RPS
   * @param fieldRelative false if robot relative
   */
  public void driveRobot(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, imu.fCDrotation2d)
            : new ChassisSpeeds(xSpeed, ySpeed, rot);
    driveRobotRelative(chassisSpeeds);          
  }

  /**
   * Drives robot based on requested robot relative axis movements
   * Required for PathPlanner
   * @param chassisSpeeds
   */
  public void  driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds( swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Calculates the current chassis speeds derived from module states.
   * Required for PathPlanner
   * @return Chassis speeds
   */
  public ChassisSpeeds getCurrentSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()); 
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public double squareJoystick(double joystickIn) {
    return Math.signum(joystickIn) * joystickIn * joystickIn;
  }

  //  ======================  Tracking Commands

  // override heading wjile tracking and just a bit longer
  public Optional<Rotation2d> getRotationTargetOverride(){
    // Some condition that should decide if we want to override rotation
    if(Globals.getNoteTracking() && Globals.noteTarget.valid) {
        // Return an optional containing the rotation override (this should be a field relative rotation)
        Rotation2d correctedHeading = Rotation2d.fromDegrees(normalizeHeadingDeg(imu.headingDeg - Globals.noteTarget.bearingDeg)) ;
        lastHeadingOverride = correctedHeading;
        trackTimer.reset();
        return Optional.of(correctedHeading);
    } else {
        // return an empty optional when we don't want to override the path's rotation
        //  keep last override for short time
        if (trackTimer.hasElapsed(1.0)) {
          lastHeadingOverride = new Rotation2d();
          return Optional.empty();
        } else {
          return Optional.of(lastHeadingOverride);
        }

    }
  }

  public  void setSpeakerTracking(boolean on){
    Globals.setSpeakerTracking(on);
  }

  public  void setNoteTracking(boolean on){
    Globals.setNoteTracking(on);
  }

  public  void setTurboMode(boolean on){
    if (on){
      speedFactor = DriveConstants.kAlexSpeedFactor;
    } else {
      speedFactor = DriveConstants.kAtleeSpeedFactor;
    }
  }

  //  ======================  Heading related utilities.

  /** Zeroes the heading of the robot. And zeros out heading for Odometry */
  public void resetHeading() {
    imu.reset();
    resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), imu.rotation2d )); 
    lockCurrentHeading();
  }
  
  public void newHeadingSetpoint(double newSetpointRad) {
    headingSetpoint = newSetpointRad;
    headingLockController.reset(imu.headingRad);
    headingLocked = true;
  }

  private double normalizeHeadingDeg(double headingDeg) {
    while (headingDeg > 180) {headingDeg -= 360;}
    while (headingDeg < 180) {headingDeg += 360;}
    return headingDeg;
  }

  public void lockCurrentHeading() {
    newHeadingSetpoint(imu.headingRad);
  }

  public boolean isNotRotating() {

    return (Math.abs(imu.yawRate) < AutoConstants.kNotRotating);
  }

  //------------------------------
    public void squareUp() {
    if (Math.abs(imu.headingRad) < Math.PI / 2) {
      newHeadingSetpoint(0);
    } else {
      newHeadingSetpoint(Math.PI);
    }
  }

  // ============ Public Command Interface  ========================================
  public Command driveCmd()                       {return runOnce(() -> drive());}

  public Command resetHeadingCmd()                {return runOnce(() -> resetHeading());}
  public Command setNoteTrackingCmd(boolean on)   {return runOnce(() -> setNoteTracking(on));}
  public Command setSpeakerTrackingCmd(boolean on){return runOnce(() -> setSpeakerTracking(on));}
  public Command setTurboModeCmd(boolean on)      {return runOnce(() -> setTurboMode(on));}
  public Command setXCmd()                        {return runOnce(() -> setX());}

}
