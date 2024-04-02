package frc.robot.utils;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IMUInterface{

    public double      headingRad = 0;
    public double      headingDeg = 0;
    public double      pitch = 0;
    public double      roll = 0;
    public double      yawRate = 0;
    
    private double     fCDOffset  = 0;
    private final AHRS m_robotIMU = new AHRS(SPI.Port.kMXP);

    public IMUInterface() {
    }
  
    public void update(){
          
        headingDeg    = -m_robotIMU.getAngle();
        headingRad    = Math.toRadians(headingDeg);
        pitch         = -m_robotIMU.getPitch();
        roll          = -m_robotIMU.getRoll();
        yawRate       = m_robotIMU.getRate();

        SmartDashboard.putString("Robot Heading", String.format("%.1f", headingDeg));
    }

    public void reset() {
        m_robotIMU.reset();
        Globals.gyroHasBeenReset = true;
        setFieldOrientation();
        update();
    }

    public void setFieldOrientation() {
        if (DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == Alliance.Red)){
            m_robotIMU.setAngleAdjustment(180);
            fCDOffset = Math.PI;
        } else {
            m_robotIMU.setAngleAdjustment(0);
            fCDOffset = 0;
        }
    }

    public void setAngleOffset(double offsetDeg){
        m_robotIMU.setAngleAdjustment(-offsetDeg);  // Native IMU has negative rotation which is reversed elsewhere
    }

    public Rotation2d getFCDRotation2d(){
        return Rotation2d.fromRadians(Math.IEEEremainder(headingRad + fCDOffset, Math.PI * 2));
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromRadians(headingRad);
    }

}
