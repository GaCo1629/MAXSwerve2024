package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IMUInterface{

    public double      headingRad = 0;
    public double      headingDeg = 0;
    public double      fCDheadingRad = 0;
    public double      pitch = 0;
    public double      roll = 0;
    public double      yawRate = 0;
    public Rotation2d  rotation2d = new Rotation2d();
    public Rotation2d  fCDrotation2d = new Rotation2d();
  
    private double m_gyro2FieldOffset = 0;
    private final AHRS m_robotIMU = new AHRS(SPI.Port.kMXP);

    public IMUInterface() {
     
    }
  
    public void update(){
          
        double angle = -m_robotIMU.getAngle();
        
        headingRad    = Math.IEEEremainder(Math.toRadians(angle) + m_gyro2FieldOffset, Math.PI * 2);
        headingDeg    = Math.toDegrees(headingRad);
        // <ust adjust Field Centric driving if starting pointing backwards
        // fCDheading = Math.IEEEremainder(Math.toRadians(angle) + Math.PI, Math.PI * 2);
        fCDheadingRad = headingRad;
        pitch      = -m_robotIMU.getPitch();
        roll       = -m_robotIMU.getRoll();
        yawRate    = m_robotIMU.getRate();

        rotation2d = Rotation2d.fromRadians(headingRad);
        fCDrotation2d = Rotation2d.fromRadians(fCDheadingRad);

        Globals.robotPitch = pitch;
        Globals.robotRoll  = roll;

        SmartDashboard.putNumber("Robot Heading", Math.toDegrees(headingRad));
        SmartDashboard.putNumber("Robot Pitch", pitch);
        SmartDashboard.putNumber("Robot roll", roll);
        SmartDashboard.putNumber("Robot rate", Math.toDegrees(yawRate));
    }

    public void reset() {
        m_robotIMU.reset();
        Globals.gyroHasBeenReset = true;
        setFieldOrientation();
        update();
    }

    public void setFieldOrientation() {
        if (DriverStation.getAlliance().get() == Alliance.Red){
            m_gyro2FieldOffset = Math.PI;
        } else {
            m_gyro2FieldOffset = 0.0;  
        }
    }
}
