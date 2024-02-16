package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BatonConstants;
import frc.robot.Constants.TiltConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkAnalogSensor;

public class BatonSubsystem extends SubsystemBase {
    private CANSparkMax intake            = null;
    //private MAXTilt     tiltLeft          = null;
    //private MAXTilt     tiltRight         = null;
    private FLEXShooter shooterTop    ;
    private FLEXShooter shooterBot    ;

    private AbsoluteEncoder tiltEncoder   = null;
    

    private double tiltAngleSetPoint;
    private double leftTiltAngle;
    private double rightTiltAngle;
    private double shooterSpeedSetPoint;
    private double shooterSpeedTop;
    private double shooterSpeedBot;
    private double noteSensor;
    private BatonState currentState = BatonState.IDLE;

     private final SparkAnalogSensor   m_rangeFinder; //////////

    //private PS4Controller driver;
    private Joystick copilot_1;
    //private Joystick copilot_2;

    public BatonSubsystem (PS4Controller driver, Joystick copilot_1, Joystick copilot_2){
        //this.driver = driver;
        this.copilot_1 = copilot_1;
        //this.copilot_2 = copilot_2;

        intake = new CANSparkMax(BatonConstants.intakeID, MotorType.kBrushless);
        
        //tiltLeft  = new MAXTilt("Left Pos",    BatonConstants.tiltLeftID, false);
        //tiltRight = new MAXTilt("Right Pos",   BatonConstants.tiltRightID, true);

        shooterBot = new FLEXShooter("Bottom", BatonConstants.shooterBotID, true);
        shooterTop = new FLEXShooter("Top",    BatonConstants.shooterTopID, false);

        m_rangeFinder = intake.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
        shooterSpeedSetPoint = 0;
        tiltAngleSetPoint    = TiltConstants.homeAngle;
    }

    @Override
    public void periodic() { 

       // leftTiltAngle   = tiltLeft.getAngle();
       // rightTiltAngle  = tiltRight.getAngle();

        shooterSpeedBot = shooterBot.getRPM();
        shooterSpeedTop = shooterTop.getRPM();

        shooterBot.setRPM(shooterSpeedSetPoint);
        shooterTop.setRPM(shooterSpeedSetPoint);
        noteSensor = m_rangeFinder.getVoltage();

        runStateMachine();

        SmartDashboard.putNumber("tilt setpoint", tiltAngleSetPoint);
        SmartDashboard.putNumber("left tilt angle",  leftTiltAngle);
        SmartDashboard.putNumber("right tilt angle", rightTiltAngle);

        SmartDashboard.putNumber("shooter setpoint", shooterSpeedSetPoint);
        SmartDashboard.putNumber("shooter bottom RPM", shooterSpeedBot);
        SmartDashboard.putNumber("shooter top RPM", shooterSpeedTop);

        SmartDashboard.putNumber("Intake Range", m_rangeFinder.getVoltage());
    }

    public void runStateMachine(){
        switch (currentState) {
            case IDLE:
                break;

            case COLLECTING:
                 if (noteSensor > BatonConstants.seeingNote){
                    stopCollector();
                    currentState = BatonState.HOLDING;
                 }
                break;
        
            case HOLDING:
                 
                break;
                
            case SHOOTING:
                if (noteSensor < BatonConstants.seeingNote){
                    stopCollector();
                    stopShooter();
                    currentState = BatonState.IDLE;
                }
                 break;

            default:
                break;
        }
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
        currentState = BatonState.COLLECTING;
    }
    public Command collectCmd() {return this.runOnce(() -> collect());}

    public void eject (){
        intake.set(BatonConstants.eject);
    }
    public Command ejectCmd() {return this.runOnce(() -> eject());}

    public void fire (){
        if (  (shooterSpeedSetPoint > 10) && (Math.abs(shooterSpeedSetPoint - shooterSpeedBot) < 20)     ){
            intake.set(BatonConstants.fire);
            currentState = BatonState.SHOOTING;
        } 
    }
    public Command fireCmd() {return this.runOnce(() -> fire());}

    public void stopCollector (){
       intake.set(BatonConstants.stopCollector);
    }
    public Command stopCollectorCmd() {return this.runOnce(() -> stopCollector());}

    public void toggleShooter(double rpm) {
        if (shooterSpeedSetPoint > 0) {
            setShooterRPM(0);
        } else {
            setShooterRPM(rpm); 
        }
    }
    public Command toggleShooterCmd(double speed) {return this.runOnce(() -> toggleShooter(speed));}

    public void stopShooter(){
        setShooterRPM(0);
    }
}
