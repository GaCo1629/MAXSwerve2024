// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Point;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.BatonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.utils.BackImageSource;
import frc.robot.utils.FrontImageSource;
import frc.robot.utils.GPIDController;
import frc.robot.utils.Globals;
import frc.robot.utils.IMUInterface;
import frc.robot.utils.LEDmode;
import frc.robot.utils.MAXSwerveModule;
import frc.robot.utils.PassSource;
import frc.robot.utils.Target;
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
  private double  speedFactor = DriveConstants.kAtleeSpeedFactor;

  private SlewRateLimiter XLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter YLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);

  private ProfiledPIDController headingLockController;
  private GPIDController         trackingController;
  
  private Timer       trackTimer = new Timer();
  
  // Odometry class for tracking robot pose (use pose estimator for ading vision)
  SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics,
    imu.getRotation2d(),
    new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition(),  m_rearRight.getPosition()
    },
    new Pose2d());
 

  /** Creates a new DriveSubsystem. */
   public DriveSubsystem(PS4Controller driver, PS4Controller copilot_1, Joystick copilot_2) {
    this.driver = driver;
    //this.copilot_1 = copilot_1;
    //this.copilot_2 = copilot_2;

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::ppResetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
              new PIDConstants(8.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(4.0, 0.0, 0.0), // Rotation PID constants
              4.5, // Max module speed, in m/s
              0.409, // Drive base radius in meters. 16.125" Distance from robot center to furthest module.
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
    headingLockController.setTolerance(AutoConstants.kDHeadingLockTollerance);
    
    trackingController = new GPIDController(AutoConstants.kPTrackingController, 
                                                      AutoConstants.kITrackingController, 
                                                      AutoConstants.kDTrackingController);
    trackingController.enableContinuousInput(-180, 180);
    trackingController.setTolerance(AutoConstants.kToleranceTrackingController);
    trackingController.setOutputRange(-AutoConstants.kOutputLimitTrackingController, AutoConstants.kOutputLimitTrackingController);
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
    Globals.setAmplifying(false);
    VisionSubsystem.setBackImageSource(BackImageSource.SPEAKER);
    VisionSubsystem.setFrontImageSource(FrontImageSource.NOTE);

    trackTimer.start();
    Globals.setLEDMode(LEDmode.SPEEDOMETER);
    trackingController.calculate(0,0);
  }


  @Override
  public void periodic() {

    // Update the odometry in the periodic block
    imu.update();
    odometry.update(
      imu.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()}
    );
    
    Globals.robotAtHeading = trackingController.atSetpoint();

    Globals.driveSubsystemFaults = getFaults();
    SmartDashboard.putBoolean("Drive Fault", Globals.driveSubsystemFaults != 0);
    SmartDashboard.putString("Drive Faults", String.format("%s", Integer.toBinaryString(Globals.driveSubsystemFaults)));

    SmartDashboard.putBoolean("At Heading", Globals.robotAtHeading);
    SmartDashboard.putNumber("Heading Error", trackingController.getPositionError());

    // Display Estimated Position
    SmartDashboard.putString("Estimated Pos", odometry.getEstimatedPosition().toString());

    getSpeakerTargetFromOdometry();
    getPassTargetFromOdometry();
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
     pose.getRotation(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      pose);
  }

  /**
   * Version of ResetOdometry which records the fact that Path Planner has set the robot location
   * @param pose new pose to be assigned to robot
   */
  public void ppResetOdometry(Pose2d pose) {
    Globals.startingLocationSet = true;

    // skew the angle returned by the IMU
    imu.setAngleOffset(pose.getRotation().getDegrees());
    resetOdometry(pose);
    newHeadingSetpoint(pose.getRotation().getRadians());
  }

  /**
   * Drive Method for Teleop ------------------------------------------
   */
  public void driveTeleop() {
    
    double xSpeed;
    double ySpeed;
    double rotate;
    boolean fieldRelative = true;

    // Read joystick values
    xSpeed     = squareJoystick(-MathUtil.applyDeadband(driver.getLeftY(), OIConstants.kDriveDeadband)) *  speedFactor;
    ySpeed     = squareJoystick(-MathUtil.applyDeadband(driver.getLeftX(), OIConstants.kDriveDeadband)) *  speedFactor;
    rotate     = squareJoystick(-MathUtil.applyDeadband(driver.getRightX(), OIConstants.kDriveDeadband)) * DriveConstants.kAtleeTurnFactor;
 
    SmartDashboard.putBoolean("Defense", Globals.defenseActive);
    if (Globals.defenseActive) {
      XLimiter.reset(xSpeed); 
      YLimiter.reset(ySpeed) ;
    } else {
      xSpeed = XLimiter.calculate(xSpeed);
      ySpeed = YLimiter.calculate(ySpeed);
    }
  
    // TARGET TRACKING =======================================================

    if (Globals.getSpeakerTracking()) {  // --- TRACKING SPEAKER  ---------------------
      lockCurrentHeading();  // prepare for return to heading hold

      SmartDashboard.putString("Mode", "Speaker")  ;
      VisionSubsystem.setBackImageSource(BackImageSource.SPEAKER);

      if (Globals.speakerTarget.valid) {
        
        // Calculate turn power to point to speaker.
        rotate = trackingCalculate(Globals.speakerTarget.bearingDeg + BatonConstants.offTargetShooting);
        lockCurrentHeading();  // prepare for return to heading hold

      } else if (Globals.odoTarget.valid) {
        
        rotate = trackingCalculate(imu.headingDeg - Globals.odoTarget.bearingDeg);

        SmartDashboard.putString("odo", String.format("Head %f  SP %f  Rot %f", imu.headingDeg, Globals.odoTarget.bearingDeg, rotate));
      }
      rotate += (ySpeed * 0.2);  // Add additional rotation based on robot's sideways motion 

    } else if (Globals.getNoteTracking()) {  // --- TRACKING NOTE ---------------
      lockCurrentHeading();  // prepare for return to heading hold

      SmartDashboard.putString("Mode", "Note")  ;
      VisionSubsystem.setFrontImageSource(FrontImageSource.NOTE);
      fieldRelative = false;
      ySpeed = 0;
      if (Globals.noteTarget.valid){
        // Calculate turn power to point to note.
        rotate = trackingCalculate(Globals.noteTarget.bearingDeg) * 0.70;  // was 0.75
        if (Math.abs(Globals.noteTarget.bearingDeg) < 10){
          xSpeed = Globals.noteTarget.range * 0.35; 
        } else {
          xSpeed = BatonConstants.noteApproachSpeed;
        }
      } else {
        if (Math.abs(xSpeed) < BatonConstants.noteApproachSpeed ){
          xSpeed = BatonConstants.noteApproachSpeed;
        }
        rotate = headingLockController.calculate(imu.headingRad, headingSetpoint);
      }

    }  else if (Globals.getAmplifying()) {  // --  AMPLIFYING --------------------
      SmartDashboard.putString("Mode", "AMP")  ;
      //if (Globals.batonState == BatonState.AMP_LOWERING){
      // xSpeed = -0.2;
      //}
    } else if (Globals.passingEnabled) { // -- PASSING ---------------------------
      if (Globals.passTarget.valid) {
        rotate = trackingCalculate(imu.headingDeg - Globals.passTarget.bearingDeg);
      }
    }
    
    else {  // ---  MANUAL DRIVING-------------------------------------------------

      VisionSubsystem.setBackImageSource(BackImageSource.SPEAKER);
      VisionSubsystem.setFrontImageSource(FrontImageSource.NOTE);

      // should we be in auto or not?
      if (Math.abs(rotate) > 0.02) {
        headingLocked = false;
        headingSetpoint = imu.headingRad;
      
      } else if (!headingLocked && isNotRotating()) {
        lockCurrentHeading();
      }

      if (Globals.liftIsEnabled) {
        headingLocked = false;
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

        if (Globals.defenseActive) {
          rotLimiter.reset(rotate); 
        } else {
          rotate = rotLimiter.calculate(rotate);
        }        
      }
    }

    SmartDashboard.putNumber("Rotate", rotate)  ;

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
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, imu.getFCDRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot);
    driveRobotRelative(chassisSpeeds);          
  }

  /**
   * Drives robot based on requested robot relative axis movements
   * Required for PathPlanner
   * @param chassisSpeeds
   */
  public void  driveRobotRelative(ChassisSpeeds chassisSpeeds) {

    // Save overall speed for LEDs
    Globals.speed = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds( swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void stopRobot(){
    driveRobotRelative(new ChassisSpeeds());
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
   * Sets the wheels into an || formation to make movement ready.
   */
  public void setRoll() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
    m_frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
    m_rearRight.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
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

  public void squareUp() {
    if (Math.abs(imu.headingRad) < Math.PI / 2) {
      newHeadingSetpoint(0);
    } else {
      newHeadingSetpoint(Math.PI);
    }
  }

  public  void setTurboOn(){
      speedFactor = DriveConstants.kAlexSpeedFactor;
  } 
  
  public void setTurboOff(){
      speedFactor = DriveConstants.kAtleeSpeedFactor;
  }

  // set the global odometry speaker target
  public void getSpeakerTargetFromOdometry() {
    if (Globals.startingLocationSet && DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        Globals.odoTarget = getTargetFromOdometry(FieldConstants.redSpeaker);
      } else {
        Globals.odoTarget = getTargetFromOdometry(FieldConstants.blueSpeaker);
      }
    } else {
      Globals.odoTarget = new Target();
    }
    SmartDashboard.putString("Odo Target", Globals.odoTarget.toString());
  }

  // set the global odometry pass target
  public void getPassTargetFromOdometry() {
    if (Globals.startingLocationSet && DriverStation.getAlliance().isPresent() && (Globals.passSource != PassSource.UNKNOWN)) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        if(Globals.passSource == PassSource.NEUTRAL){
          Globals.passTarget = getTargetFromOdometry(FieldConstants.redNeutral);
        } else {
          Globals.passTarget = getTargetFromOdometry(FieldConstants.redSource);
        }
      } else {
        if(Globals.passSource == PassSource.NEUTRAL){
          Globals.passTarget = getTargetFromOdometry(FieldConstants.blueNeutral);
        } else {
          Globals.passTarget = getTargetFromOdometry(FieldConstants.blueSource);
        }
      }
    } else {
      Globals.passTarget = new Target();
    }
    SmartDashboard.putString("Pass Target", Globals.passTarget.toString());
  }

  /** 
   * Calculates the range and bearing to the pont of Focus
   */
  public Target getTargetFromOdometry(Point focus) {
    Pose2d position = odometry.getEstimatedPosition();
    Point  robot    = new Point(position.getX(), position.getY());

    double dX = robot.x - focus.x ;
    double dY = robot.y - focus.y ;
    double range   = Math.hypot(dX, dY);
    double bearing = Math.atan2(dY, dX);
    
    return new Target(true, range, Math.toDegrees(bearing));
  }

  /**
   *  Called while shooting at target.   
   *  Project forward from target to calculate robot's position.
   */
  public void updateOdometryFromSpeaker() {
    Point  speaker  = new Point();
    double X;
    double Y;

    // If the speaker target is valid, select the correct speaker and offset robot location by target range, bearing.
    if (Globals.speakerTarget.valid){
      if (DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == Alliance.Red)) {
        speaker = FieldConstants.redSpeaker;
      } else {
        speaker = FieldConstants.blueSpeaker;
      }

      double bearingToTarget = Globals.speakerTarget.bearingRad + imu.headingRad;
      // double bearingToTarget = Globals.speakerTarget.bearingRad + imu.headingRad - Math.toRadian(BatonConstants.offTargetShooting);

      X = speaker.x + (Math.cos(bearingToTarget) * Globals.speakerTarget.range);
      Y = speaker.y + (Math.sin(bearingToTarget) * Globals.speakerTarget.range);
      resetOdometry(new Pose2d(X, Y, imu.getRotation2d()));
      Globals.startingLocationSet = true;
    }    
  }

  //  ======================  Heading related utilities.

  /** Zeroes the heading of the robot. And zeros out heading for Odometry */
  public void resetHeading() {
    imu.reset();
    resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), imu.getRotation2d() )); 
    lockCurrentHeading();
  }
  
  public void newHeadingSetpoint(double newSetpointRad) {
    headingSetpoint = newSetpointRad;
    headingLockController.reset(imu.headingRad);
    headingLocked = true;
    SmartDashboard.putString("heading Setpoint",  String.format("%.1f",Math.toDegrees(headingSetpoint)));
  }

  public void lockCurrentHeading() {
    newHeadingSetpoint(imu.headingRad);
  }

  public void turnToSource() {
    if (DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == Alliance.Red)){
      newHeadingSetpoint(FieldConstants.redSourceAngle);          
    } else {
      newHeadingSetpoint(FieldConstants.blueSourceAngle);          
    }
  }

  public void turnToFaceAmp() {
    if (Globals.passTarget.valid) {
      newHeadingSetpoint(Globals.passTarget.bearingRad);  
    }
  }

  public boolean isNotRotating() {
    return (Math.abs(imu.yawRate) < AutoConstants.kNotRotating);
  }

  public boolean atSetpoint() {
    return headingLockController.atGoal();
  }

  public double getHeadingDeg() {
    return imu.headingDeg;
  }

  public double trackingCalculate(double bearingDeg){
    return trackingController.calculate(bearingDeg, 0);
  }

  public double headingLockCalculate() {
      return headingLockController.calculate(imu.headingRad, headingSetpoint);
  }

  public int getFaults() {
    return m_frontLeft.getFaults() + m_frontRight.getFaults() + m_rearLeft.getFaults() + m_rearRight.getFaults();
  }

}
  

  
