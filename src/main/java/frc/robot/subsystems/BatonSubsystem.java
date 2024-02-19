package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BatonConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TiltConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkAnalogSensor;

public class BatonSubsystem extends SubsystemBase {
    private CANSparkMax intake;
    private CANSparkMax tiltLeft;
    private CANSparkMax tiltRight;
    private FLEXShooter shooterTop;
    private FLEXShooter shooterBot;
    private GPIDController tiltControl;
    private Timer       stateTimer = new Timer();

    private AbsoluteEncoder tiltEncoder;

    private double tiltAngleSetPoint;
    private double tiltAngle;
    private double tiltPower;
    private double shooterSpeedSetPoint;
    private double shooterSpeedTop;
    private double shooterSpeedBot;
    private double noteSensor;
    private BatonState currentState;

    private final SparkAnalogSensor   m_rangeFinder; 

    //private PS4Controller driver;
    private Joystick copilot_1;
    //private Joystick copilot_2;

    public BatonSubsystem (PS4Controller driver, Joystick copilot_1, Joystick copilot_2){
        //this.driver = driver;
        this.copilot_1 = copilot_1;
        //this.copilot_2 = copilot_2;

        intake = new CANSparkMax(BatonConstants.intakeID, MotorType.kBrushless);
        intake.restoreFactoryDefaults(true);
        m_rangeFinder = intake.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
        intake.burnFlash();
        
        tiltLeft  = new CANSparkMax(BatonConstants.tiltLeftID, MotorType.kBrushless);
        tiltLeft.restoreFactoryDefaults();
        tiltLeft.setIdleMode(TiltConstants.kMotorIdleMode);
        tiltLeft.setSmartCurrentLimit(TiltConstants.kMotorCurrentLimit);
        tiltLeft.burnFlash();
    

        tiltRight = new CANSparkMax(BatonConstants.tiltRightID, MotorType.kBrushless);
        tiltRight.restoreFactoryDefaults();
        tiltRight.setIdleMode(TiltConstants.kMotorIdleMode);
        tiltRight.setSmartCurrentLimit(TiltConstants.kMotorCurrentLimit);

        tiltEncoder = tiltRight.getAbsoluteEncoder(Type.kDutyCycle);
        tiltEncoder.setPositionConversionFactor(TiltConstants.kEncoderPositionFactor);
        tiltRight.burnFlash();

        tiltControl = new GPIDController(TiltConstants.kP, TiltConstants.kI, TiltConstants.kD);    
        tiltControl.setIZone(TiltConstants.kZone);
        tiltControl.setIntegralDeadband(TiltConstants.kIDeadband);
        tiltControl.setSetpoint(0);

        shooterBot = new FLEXShooter("Bot", BatonConstants.shooterBotID, true);
        shooterTop = new FLEXShooter("Top", BatonConstants.shooterTopID, false);

        init();
    }

    public void init(){
        setState(BatonState.IDLE);
        setShooterRPM(0);
        setTiltAngle(0);
    }


    @Override
    public void periodic() { 

        tiltAngle  = getSafeTiltAngle(); 
        runTiltPID(tiltAngle);

        shooterSpeedBot = shooterBot.getRPM();
        shooterSpeedTop = shooterTop.getRPM();

        shooterBot.setRPM(shooterSpeedSetPoint);
        shooterTop.setRPM(shooterSpeedSetPoint);
        noteSensor = m_rangeFinder.getVoltage();

        runStateMachine();

        SmartDashboard.putNumber("tilt setpoint", tiltAngleSetPoint);
        SmartDashboard.putNumber("tilt angle",  tiltAngle);

        SmartDashboard.putNumber("shooter setpoint", shooterSpeedSetPoint);
        SmartDashboard.putNumber("shooter bot RPM", shooterSpeedBot);
        SmartDashboard.putNumber("shooter top RPM", shooterSpeedTop);

        SmartDashboard.putNumber("Intake Range", m_rangeFinder.getVoltage());
    }

    /**
     * Baton state machine.  Manages the sequencing of intake, aim and fire.
     */
    public void runStateMachine(){
        switch (currentState) {
            case IDLE:
                // Exits by button press.
                break;

            case COLLECTING:
                if (noteSensor > BatonConstants.seeingNote){
                    stopIntake();
                    setState(BatonState.HOLDING);
                }
                break;
        
            case HOLDING:
                // Exits by button press.
                break;
                
            case SHOOTING:
                if (noteSensor < BatonConstants.seeingNote){
                    setState(BatonState.WAITING);
                }
                break;

            case WAITING:    
                if (stateTimer.hasElapsed(0.5)){
                    stopIntake();
                    stopShooter();
                    setState(BatonState.WAITING);
                }
                 break;

            default:
                break;
        }
    }

    public void  setState(BatonState state) {
        currentState = state;
        stateTimer.restart();
    }

    // ===== TILT Commands

    public void setTiltAngle(double angle){
        tiltAngleSetPoint = MathUtil.clamp(angle, TiltConstants.minEncoderPosition, TiltConstants.maxEncoderPosition);
        tiltControl.setSetpoint(tiltAngleSetPoint);
    }
    public Command setTiltAngleCmd(double angle) {return this.runOnce(() -> setTiltAngle(angle));}

    public boolean tiltInPosition() {
        boolean inPosition = (Math.abs(tiltAngleSetPoint - tiltAngle) < TiltConstants.tiltThresholdDeg);
        SmartDashboard.putBoolean("Tilt In position", inPosition);
        return inPosition;
    }

    public double getSafeTiltAngle() {
        double safe = tiltEncoder.getPosition();
        if (safe > 180) safe = 0;

        return safe;
    }

    /**
     *  Determine power to send to two tilt motors.
     *  Run the PID and then limit the output power.
     * @param currentAngle
     */
    public void runTiltPID(double currentAngle) {
        double output = tiltControl.calculate(currentAngle);

        // clip output to acceptable range
        output = MathUtil.clamp(output, TiltConstants.kMinOutput, TiltConstants.kMaxOutput);

        // if we are lowering, and are close to our target, just set power to zero to brake
        if ((output < 0) && (Math.abs(tiltControl.getPositionError()) < 5)) {
            output = 0;
        }

        //output = -copilot_1.getY() / 2;

        SmartDashboard.putNumber("Tilt Out", output);

        tiltPower = output;
        tiltLeft.set(-tiltPower);
        tiltRight.set(tiltPower);
    }

    // ===== SHOOTER Commands  ===============================

    public void setShooterRPM(double speed){
        shooterSpeedSetPoint = speed;
        shooterBot.setRPM(shooterSpeedSetPoint);
        shooterTop.setRPM(shooterSpeedSetPoint);
    }
    public Command setShooterRPMCmd(double speed) {return this.runOnce(() -> setShooterRPM(speed));}
    
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

    public boolean shooterUpToSpeed() {
        return (shooterSpeedSetPoint > 0) && (Math.abs(shooterSpeedSetPoint - shooterSpeedBot) < ShooterConstants.speedThresholdRPM);
    }

    // ===== INTAKE commands ==================

    public void collect (){
        intake.set(BatonConstants.collect);
        setState(BatonState.COLLECTING);
    }
    public Command collectCmd() {return this.runOnce(() -> collect());}

    public void fire (){
        if (shooterUpToSpeed()){
            intake.set(BatonConstants.fire);
            setState(BatonState.SHOOTING);
        } 
    }
    public Command fireCmd() {return this.runOnce(() -> fire());}

    public void stopIntake (){
       intake.set(BatonConstants.stopCollector);
    }
    public Command stopIntakeCmd() {return this.runOnce(() -> stopIntake());}

    public void eject (){
        intake.set(BatonConstants.eject);
    }
    public Command ejectCmd() {return this.runOnce(() -> eject());}

    
}
