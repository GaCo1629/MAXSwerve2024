package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BatonConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class BatonSubsystem extends SubsystemBase {
    private final CANSparkMax intake;
    private final CANSparkMax tiltLeft;
    private final CANSparkMax tiltRight;
    private final CANSparkFlex shooterTop;
    private final CANSparkFlex shooterBottom;

    private final AbsoluteEncoder tiltEncoder;
    
    private double tiltAngle;
    private double tiltAngleSetPoint;
    private double shooterSpeedSetPoint;
    private double shooterSpeedTop;
    private double shooterSpeedBottom;

    private PS4Controller driver;
    private Joystick copilot_1;
    private Joystick copilot_2;

    public BatonSubsystem (PS4Controller driver, Joystick copilot_1, Joystick copilot_2){
        this.driver = driver;
        this.copilot_1 = copilot_1;
        this.copilot_2 = copilot_2;

        intake = new CANSparkMax(BatonConstants.intakeID, MotorType.kBrushless);
        tiltLeft = new CANSparkMax(BatonConstants.tiltLeftID, MotorType.kBrushless);
        tiltRight = new CANSparkMax(BatonConstants.tiltRightID, MotorType.kBrushless);
        shooterBottom = new CANSparkFlex(BatonConstants.shooterBottomID, MotorType.kBrushless);
        shooterTop = new CANSparkFlex(BatonConstants.shooterTopID, MotorType.kBrushless);
        tiltEncoder = tiltLeft.getAbsoluteEncoder(Type.kDutyCycle);

        tiltAngle = tiltEncoder.getPosition();
        shooterSpeedSetPoint = 0;
    }

    @Override
    public void periodic() { 
        double power;

        tiltAngle = tiltEncoder.getPosition();
        shooterSpeedBottom = shooterBottom.getEncoder().getVelocity();
        shooterSpeedTop = shooterTop.getEncoder().getVelocity();

        power = shooterSpeedSetPoint / BatonConstants.maxShooterRPM;
        shooterBottom.set(power);
        shooterTop.set(-power);

        SmartDashboard.putNumber("tilt angle", tiltAngle);
        SmartDashboard.putNumber("tilt setpoint", tiltAngleSetPoint);

        SmartDashboard.putNumber("shooter setpoint", shooterSpeedSetPoint);
        SmartDashboard.putNumber("shooter power",power);   
        SmartDashboard.putNumber("shooter bottom RPM", shooterSpeedBottom);
        SmartDashboard.putNumber("shooter top RPM", shooterSpeedTop);
    }

    public void setTiltAngle(double angle){
        tiltAngleSetPoint = angle;
    }
    public Command setTiltAngleCmd(double angle) {return this.runOnce(() -> setTiltAngle(angle));}

    public void setShooterRPM(double speed){
        shooterSpeedSetPoint = speed;
    }
    public Command setShooterRPMCmd(double speed) {return this.runOnce(() -> setShooterRPM(speed));}
    
    public void collect (){
        intake.set(BatonConstants.collect);
    }
    public Command collectCmd() {return this.runOnce(() -> collect());}

    public void eject (){
        intake.set(BatonConstants.eject);
    }
    public Command ejectCmd() {return this.runOnce(() -> eject());}

    public void fire (){
        intake.set(BatonConstants.fire);   
    }
    public Command fireCmd() {return this.runOnce(() -> fire());}

    public void stopCollector (){
        intake.set(BatonConstants.stopCollector);
    }
    public Command stopCollectorCmd() {return this.runOnce(() -> stopCollector());}

    

}
