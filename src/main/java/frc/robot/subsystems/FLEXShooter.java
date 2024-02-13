// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;

public class FLEXShooter {
  private final CANSparkFlex    m_SparkFlex;
  private final RelativeEncoder m_Encoder;
  private final SparkPIDController m_PIDController;
  private boolean m_invert = false;
  
  /**
   * Constructs a Shooter module and configures the motor, encoder, and PID controller. 
   */
  public FLEXShooter(int CANId, boolean invert) {
    m_invert = invert;
    m_SparkFlex = new CANSparkFlex(CANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARK FLEX to a known state before configuring
    m_SparkFlex.restoreFactoryDefaults();

    // Setup encoder and PID controller
    m_Encoder = m_SparkFlex.getEncoder();
    m_PIDController = m_SparkFlex.getPIDController();
    m_PIDController.setFeedbackDevice(m_Encoder);

    // Set the PID gains for the motor.
    m_PIDController.setP(ShooterConstants.kShooterP);
    m_PIDController.setI(ShooterConstants.kShooterI);
    m_PIDController.setD(ShooterConstants.kShooterD);
    m_PIDController.setFF(ShooterConstants.kShooterFF);
    m_PIDController.setOutputRange(ShooterConstants.kShooterMinOutput,  ShooterConstants.kShooterMaxOutput);
    stop();
    
    m_SparkFlex.setIdleMode(ShooterConstants.kShooterMotorIdleMode);
    m_SparkFlex.setSmartCurrentLimit(ShooterConstants.kShooterMotorCurrentLimit);
    
    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_SparkFlex.burnFlash();
  }

   /**
   * Returns the current RPM of the module.
   *
   * @return The current RPM of the module.
   */
  public double getRPM() {
    return m_Encoder.getVelocity() * (m_invert ? -1 : 1);
  }

  /**
   * Sets the desired velocity for the module.
   *
   * @param desiredRPM Desired RPM of the shooter.
   */
  public void setRPM(double  desiredRPM) {
    // Command shooter motor to respective setpoint.
    m_PIDController.setReference(desiredRPM * (m_invert ? -1 : 1), CANSparkFlex.ControlType.kVelocity);
  }

  public void stop() {
    setRPM(0);
  }
}
