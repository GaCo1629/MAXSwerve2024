// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final double kAtleeSpeedFactor     = 0.6;
    public static final double kAtleeTurnFactor      = 0.5;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 2.0; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kFrontRightDrivingCanId = 13;
    public static final int kRearLeftDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearLeftTurningCanId = 16;
    public static final int kRearRightTurningCanId = 18;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class ShooterConstants { 

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kP = 0.0002;
    public static final double kI = 0.000001;
    public static final double kD = 0;
    public static final double kShooterFF = 1 / FlexMotorConstants.kFreeSpeedRpm;
    public static final double kZone = 100;
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;

    public static final IdleMode kMotorIdleMode = IdleMode.kCoast;
    public static final int kMotorCurrentLimit = 50; // amps
    public static final double speedThresholdRPM = 20;
  }

  public static final class TiltConstants { 
    // Calculations required for driving motor conversion factors and feed forward
    public static final double kP = 0.010;
    public static final double kI = 0.030;
    public static final double kD = 0.00;
    public static final double kFF = 0;
    public static final double kZone = 5;    
    public static final double kIDeadband = 0.5;
    public static final double kMinOutput = -0.01;
    public static final double kMaxOutput = 0.3;

    public static final double minEncoderPosition = 0.0;
    public static final double maxEncoderPosition = 90.0;
    public static final double homeAngle    = 0;
    public static final double tiltThresholdDeg = 1.0;

    public static final IdleMode kMotorIdleMode = IdleMode.kBrake;
    public static final int kMotorCurrentLimit = 40; // amps

    public static final double kEncoderPositionFactor = 360; // degrees
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoPilotController1Port= 1;
    public static final int kCoPilotController2Port= 2;
 
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {

    public static final double kNotRotating = 0.1;  // Radian per second

    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kAutoMaxAngularSpeedRPS = 6;
    public static final double kAutoMaxAngularAccelerationRPS2 = 6 ;

    // Used in Teleop Heading lock Command
    public static final double kPHeadingLockController = 1.2;
    public static final double kIHeadingLockController = 0;
    public static final double kDHeadingLockController = 0; // try to slow down approach

    public static final TrapezoidProfile.Constraints kHeadingLockConstraints = new TrapezoidProfile.Constraints(
      kAutoMaxAngularSpeedRPS, kAutoMaxAngularAccelerationRPS2);
  
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // Used in Teleop Heading lock Command
    public static final double kPTrackingController = 0.015; //  0.015
    public static final double kITrackingController = 0;
    public static final double kDTrackingController = 0; // try to slow down approach

    public static final TrapezoidProfile.Constraints kTrackingConstraints = new TrapezoidProfile.Constraints(
      kAutoMaxAngularSpeedRPS, kAutoMaxAngularAccelerationRPS2);
  
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaTrackingConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class FlexMotorConstants {
    public static final double kFreeSpeedRpm = 6500;
  }

  public class VisionConstants {
    public static double STANDARD_DEVIATION_ODOMETRY = 0.1;
    public static double STANDARD_DEVIATION_ODOMETRY_ANGLE = 0.01;
    public static double STANDARD_DEVIATION_VISION2D = 0.9;
    public static double STANDARD_DEVIATION_VISION_ANGLE = 010;
  }
  
  public class BatonConstants {
    
    public static int shooterTopID = 20;
    public static int shooterBotID = 21;
    public static int tiltLeftID = 22;
    public static int tiltRightID = 23;
    public static int intakeID = 24;
    public static double maxShooterRPM = 6500;
    public static double collect = -0.5;
    public static double fire = -1;
    public static double eject = 0.3;

    public static double stopCollector = 0;
    public static double seeingNote = 0.6;
  }

  public class LiftConstants{
    public static int leftLiftID = 30;
    public static int rightLiftID = 31;
  }
}
