// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;

import frc.robot.Constants.ShooterConstants;

public class FLEXShooter {
  private final CANSparkFlex        m_SparkFlex;
  private final RelativeEncoder     m_Encoder;
  private final SparkPIDController  m_PIDController;
  private final SparkAnalogSensor   m_rangeFinder;

  private boolean m_invert = false;
  private String  m_name;
  private double  m_setpoint = 0;
  
  /**
   * Constructs a Shooter module and configures the motor, encoder, and PID controller. 
   */
  public FLEXShooter(String name, int CANId, boolean invert) {
    m_name = name;
    m_invert = invert;
    m_SparkFlex = new CANSparkFlex(CANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARK FLEX to a known state before configuring
    m_SparkFlex.restoreFactoryDefaults();

    m_rangeFinder = m_SparkFlex.getAnalog(SparkAnalogSensor.Mode.kAbsolute);

    // Setup encoder and PID controller
    m_Encoder = m_SparkFlex.getEncoder();
    m_PIDController = m_SparkFlex.getPIDController();
    m_PIDController.setFeedbackDevice(m_Encoder);

    // Set the PID gains for the motor.
    m_PIDController.setP(ShooterConstants.kP);
    m_PIDController.setI(ShooterConstants.kI);
    m_PIDController.setD(ShooterConstants.kD);
    m_PIDController.setFF(ShooterConstants.kShooterFF);
    m_PIDController.setIZone(ShooterConstants.kZone);
    m_PIDController.setOutputRange(ShooterConstants.kMinOutput,  ShooterConstants.kMaxOutput);
    stop();
    
    m_SparkFlex.setIdleMode(ShooterConstants.kMotorIdleMode);
    m_SparkFlex.setSmartCurrentLimit(ShooterConstants.kMotorCurrentLimit);
    
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
    double vel = m_Encoder.getVelocity() * (m_invert ? -1 : 1);
    SmartDashboard.putNumber(m_name + " RPM", vel);
    SmartDashboard.putNumber(m_name + " Error", m_setpoint - vel);
    return vel;
  }

  /**
   * Sets the desired velocity for the module.
   *
   * @param desiredRPM Desired RPM of the shooter.
   */
  public void setRPM(double  desiredRPM) {
    // Command shooter motor to respective setpoint.
    m_setpoint = desiredRPM;
    m_PIDController.setReference(m_setpoint * (m_invert ? -1 : 1), CANSparkFlex.ControlType.kVelocity);
  }

  public double getVoltage (){
    double voltage = m_rangeFinder.getVoltage();
    SmartDashboard.putNumber(m_name + " Voltage", voltage);
    return voltage;
  }

  public void stop() {
    m_SparkFlex.set(0);
  }

  public short getFaults() {
    return m_SparkFlex.getFaults();
  }
}
