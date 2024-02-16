// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.TiltConstants;

public class MAXTilt {
  private final CANSparkMax        m_tiltSparkMax;
  private final RelativeEncoder    m_tiltEncoder;
  private final SparkPIDController m_tiltPIDController;
  
  private String m_name;
  //private double m_tiltOffset = TiltConstants.homeAngle;
  private double m_positionSetpoint = 0;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXTilt(String name, int tiltCANId, boolean invert) {
    m_name = name;
    m_tiltSparkMax = new CANSparkMax(tiltCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_tiltSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the tilt SPARKS MAX.
    m_tiltEncoder = m_tiltSparkMax.getEncoder();
    m_tiltPIDController = m_tiltSparkMax.getPIDController();
    m_tiltPIDController.setFeedbackDevice(m_tiltEncoder);

    // Apply position and velocity conversion factors for the tilt encoder. We
    m_tiltEncoder.setPositionConversionFactor(TiltConstants.kEncoderPositionFactor);
    m_tiltEncoder.setVelocityConversionFactor(TiltConstants.kEncoderVelocityFactor);
    

    // Set the PID gains for the tilt motor.
    m_tiltPIDController.setP(TiltConstants.kP);
    m_tiltPIDController.setI(TiltConstants.kI);
    m_tiltPIDController.setD(TiltConstants.kD);
    m_tiltPIDController.setFF(TiltConstants.kFF);
    m_tiltPIDController.setOutputRange(TiltConstants.kMinOutput, TiltConstants.kMaxOutput);

    m_tiltSparkMax.setIdleMode(TiltConstants.kMotorIdleMode);
    m_tiltSparkMax.setSmartCurrentLimit(TiltConstants.kMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_tiltSparkMax.burnFlash();

    m_tiltEncoder.setPosition(0);
  }

  
  /**
   * Returns the current angle in degrees.
   *
   * @return The current angle in degrees.
   */
  public double getAngle() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return getPosition();
  }

  public double getPosition() {
    double position = m_tiltEncoder.getPosition();

    SmartDashboard.putNumber(m_name + " POS", position);
    SmartDashboard.putNumber(m_name + " Error", m_positionSetpoint - position);
    return position;
  }

  /**
   * Returns the current setpoint in degrees.
   *
   * @return The current setpoint.
   */
  public double getSetPoint() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return m_positionSetpoint;
  }

  /**
   * Sets the desired angle in degrees
   *
   * @param angle Desired angle in degrees
   */
  public void setAngle(double angle) {

    // Apply chassis angular offset to the desired angle.
    m_positionSetpoint = angle;

   // m_positionSetpoint = Math.max(TiltConstants.maxEncoderPosition, Math.min(m_positionSetpoint, TiltConstants.minEncoderPosition));
    m_tiltPIDController.setReference(m_positionSetpoint, CANSparkMax.ControlType.kPosition);
  }

  /** Zeroes the tilt encoder. */
  public void resetEncoders() {
    m_tiltEncoder.setPosition(0);
  }
}
