// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.annotation.Target;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.utils.SwerveUtils;
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
  //  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP); 
  
  private PS4Controller driver;
  private Joystick copilot_1;
  private Joystick copilot_2;
  
  private boolean m_headingLocked = false;
  private boolean m_targetTracking = false;
  private boolean m_lastTargetTracking = false;
  
  private double  m_headingSetpoint = 0;
  private double  m_currentHeading = 0;

  private double gyro2FieldOffset = 0;


  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private ProfiledPIDController headingLockController;
  private ProfiledPIDController trackingController;
  

  // Odometry class for tracking robot pose (use pose estimator for ading vision)
  SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics,
    getRotation2d(),
    new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    },
    new Pose2d());
 

  /** Creates a new DriveSubsystem. */
   public DriveSubsystem(PS4Controller driver, Joystick copilot_1, Joystick copilot_2) {
    this.driver = driver;
    this.copilot_1 = copilot_1;
    this.copilot_2 = copilot_2;

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
    
    trackingController = new ProfiledPIDController(AutoConstants.kPTrackingController, 
                                                      AutoConstants.kITrackingController, 
                                                      AutoConstants.kDTrackingController, 
                                                      AutoConstants.kTrackingConstraints );
    trackingController.enableContinuousInput(-Math.PI, Math.PI);
    
  }

  /**
   * Initialoze everything needed when Teleop first starts up.
   */
  public void init() {

    if (!Globals.gyroReset) {
      resetHeading();
    }

    setFieldOffsets();
    lockCurrentHeading();
  }


  @Override
  public void periodic() {

    // Update the odometry in the periodic block
    m_odometry.update(
      getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()}
    );

    // Look for AprilTags on the Speakers to update static position when disabled.
    if (DriverStation.isDisabled()) {
      LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
      if (llresults.targetingResults.valid) {
        // We have a target and we're Disabled, so Update Odometry
        Pose2d robotPosition = llresults.targetingResults.getBotPose2d_wpiBlue();

        if ((robotPosition.getX() != 0) && (robotPosition.getY() != 0)) {
          SmartDashboard.putString("BotPose", robotPosition.toString());
          m_odometry.addVisionMeasurement(robotPosition, Timer.getFPGATimestamp());
        } else {
          SmartDashboard.putString("BotPose", "No Targets");
        }
      }
   }

    // Display Estimated Position
    SmartDashboard.putString("Estimated Pos", m_odometry.getEstimatedPosition().toString());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
      getRotation2d(),
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
    
    double xSpeed, xSpeedCommanded;
    double ySpeed, ySpeedCommanded;
    double rotate;
    boolean fieldRelative = true;
    boolean rateLimit = true;
    VisionTarget target;

    xSpeed     = squareJoystick(-MathUtil.applyDeadband(driver.getLeftY(), OIConstants.kDriveDeadband) *  DriveConstants.kSpeedFactor);
    ySpeed     = squareJoystick(-MathUtil.applyDeadband(driver.getLeftX(), OIConstants.kDriveDeadband) * DriveConstants.kSpeedFactor);
    rotate     = squareJoystick(-MathUtil.applyDeadband(driver.getRightX(), OIConstants.kDriveDeadband) * DriveConstants.kTurnFactor);
 
    getHeading();
    m_targetTracking = driver.getL1Button();

    target = getTarget();
    SmartDashboard.putString("Target", target.toString());

    if (m_targetTracking) {
      rotate = trackingController.calculate(target.bearing, 0);
      
      if (!m_lastTargetTracking) {
        ;//.reset();    
      }

    } else {

      
      
      // determine if heading lock should be engaged
      if (rotate != 0) {
        m_headingLocked = false;
      } else if (!m_headingLocked && isNotRotating()) {
        lockCurrentHeading();
      }

      // if Heading lock is engaged, override the user input with data from PID
      if (m_headingLocked) {
        rotate = headingLockController.calculate(m_currentHeading, m_headingSetpoint);
        if (Math.abs(rotate) < 0.025) {
          rotate = 0;
        } 
      }
      

    }

    m_lastTargetTracking = m_targetTracking;

    
    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rotate);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rotate;
    }


    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds( swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
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

  //  ======================  Vision processing
  public VisionTarget getTarget() {
    double x,y,z = 0;
    double range = 0;
    double bearing = 0;
    double elevation = 0;

    Pose3d targetLocation = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
    
    x = targetLocation.getX();
    y = targetLocation.getZ();
    z = -targetLocation.getY();

    // SmartDashboard.putString("Target Coord", String.format("X:Y:Z %5.2f  %5.2f  %5.2f ", x,y,z));

    range = Math.hypot(x, y);
    bearing = -Math.atan2(x, y);
    elevation = Math.atan2(z, range);

    if (range > 0.5) {
      return new VisionTarget(true, range, Math.toDegrees(bearing), Math.toDegrees(elevation));
    }
    
    return new VisionTarget();
  }

  //  ======================  Heading related utilities.

  /** Zeroes the heading of the robot. And zeros out heading for Odometry */
  public void resetHeading() {
    m_gyro.reset();
    Globals.gyroReset = true;

    setFieldOffsets();
    resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), getRotation2d() )); 
    lockCurrentHeading();
  }
  

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public void newHeadingSetpoint(double newSetpoint) {
    m_headingSetpoint = newSetpoint;
    headingLockController.reset(m_currentHeading);
    m_headingLocked = true;
  }

  public void lockCurrentHeading() {
    newHeadingSetpoint(getHeading());
  }

  public boolean isNotRotating() {

    return (Math.abs(m_gyro.getRate()) < AutoConstants.kNotRotating);
  }

  //------------------------------
  public void setFieldOffsets() {
    if (DriverStation.getAlliance().get() == Alliance.Red){
      gyro2FieldOffset = 0.0;
    } else {
      gyro2FieldOffset = Math.PI;  
    }
  }

  public void squareUp() {
    if (Math.abs(getHeading()) < Math.PI / 2) {
      newHeadingSetpoint(0);
    } else {
      newHeadingSetpoint(Math.PI);
    }
  }

  /***
   * Reads heading from gyro, adjusts for field orientation and sets current Heading member.
   * @return
   */
  public double getHeading() {
    m_currentHeading = Math.IEEEremainder(Math.toRadians(-m_gyro.getAngle()) + gyro2FieldOffset, Math.PI * 2);
    return m_currentHeading;
  }

  public double getPitch() {
    return -m_gyro.getRoll();
  }

  public double getRoll() {
    return -m_gyro.getPitch();
  }
  
  public double getFCDHeading() {
      return Math.IEEEremainder(Math.toRadians(-m_gyro.getAngle()) + Math.PI, Math.PI * 2);
  }

  public Rotation2d getRotation2d() {
      return Rotation2d.fromRadians(getHeading());
  }

  public Rotation2d getFCDRotation2d() {
      return Rotation2d.fromRadians(getFCDHeading());
  }


}
