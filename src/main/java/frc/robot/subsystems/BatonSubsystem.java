package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
    private AbsoluteEncoder tiltEncoder;
    private Timer       stateTimer = new Timer();
    
    private double tiltAngleSetPoint;
    private double currentTiltAngle;
    private double tiltPower;
    private double shooterSpeedSetPoint;
    private double shooterSpeedTop;
    private double shooterSpeedBot;
    private double noteSensor;
    private BatonState currentState;

    private final SparkAnalogSensor   rangeFinder; 

    private PS4Controller driver;
    //private Joystick copilot_1;
    //private Joystick copilot_2;

    public BatonSubsystem (PS4Controller driver, Joystick copilot_1, Joystick copilot_2){
        this.driver = driver;
        //this.copilot_1 = copilot_1;
        //this.copilot_2 = copilot_2;

        intake = new CANSparkMax(BatonConstants.intakeID, MotorType.kBrushless);
        intake.restoreFactoryDefaults(true);
        rangeFinder = intake.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
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
        tiltControl.setIntegratorRange(TiltConstants.kIMin,TiltConstants.kIMax);
        tiltControl.setSetpoint(0);

        shooterBot = new FLEXShooter("Bot", BatonConstants.shooterBotID, true);
        shooterTop = new FLEXShooter("Top", BatonConstants.shooterTopID, false);

        init();
    }

    public void init(){
        setState(BatonState.IDLE);
        setShooterRPM(0);
        setTiltAngle(0);
        intake.set(BatonConstants.stopCollector);
    }

    @Override
    public void periodic() { 

        // Read baton sensors
        currentTiltAngle    = getSafeTiltAngle(); 
        noteSensor          = getNoteSensorValue();
        shooterSpeedBot     = shooterBot.getRPM();
        shooterSpeedTop     = shooterTop.getRPM();

        if (Globals.speakerTrackingEnabled) {
            if  (Globals.speakerTarget.valid) {
                setTiltAngle(rangeToAngle(Globals.speakerTarget.range) + 1.250); 
                setShooterRPM(rangeToRPM(Globals.speakerTarget.range));
            }
        } else {
            setTiltAngle(0);
            setShooterRPM(0);
        }

        runTiltPID();
        runStateMachine();

        SmartDashboard.putNumber("Intake Range",    rangeFinder.getVoltage());

        SmartDashboard.putNumber("tilt setpoint",   tiltAngleSetPoint);
        SmartDashboard.putNumber("tilt angle",      currentTiltAngle);
        SmartDashboard.putNumber("tilt Power",      tiltRight.getOutputCurrent());

        SmartDashboard.putNumber("shooter setpoint", shooterSpeedSetPoint);
        SmartDashboard.putNumber("shooter bot RPM", shooterSpeedBot);
        SmartDashboard.putNumber("shooter top RPM", shooterSpeedTop);
        SmartDashboard.putString("BatonState", currentState.toString());
     
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
                if (noteInIntake()){
                    stopIntake();
                    setState(BatonState.HOLDING);
                    Globals.setNoteTracking(false);
                }
                break;
        
            case HOLDING:
                // Exits by button press.
                break;
                
            case AUTO_SHOOT:
                // Exits by being on target
                if (readyToShoot()){
                   intake.set(BatonConstants.fire);
                   setState(BatonState.SHOOTING); 
                } else if (!noteInIntake() && stateTimer.hasElapsed(2)){
                    intake.set(BatonConstants.seeingNote);
                    Globals.setSpeakerTracking(false);
                    setState(BatonState.IDLE);                     
                }
                break;
                
            case SHOOTING:
                if (!noteInIntake()){
                    setState(BatonState.WAITING);
                }
                break;

            case WAITING:    
                if (stateTimer.hasElapsed(0.5)){
                    stopIntake();
                    stopShooter();
                    setState(BatonState.IDLE);
                    Globals.setSpeakerTracking(false);
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

    public BatonState  getState() {
        return currentState;
    }

    // ===== Conversions

    public double rangeToAngle(double range) {
        //double angle = (-3.558 * range * range) + (31.335 * range) - 30.389;
        double angle = (-3.025 * range * range) + (28.00 * range) - 26.311;
        return angle;
    }

    public double rangeToRPM(double range) {
        double speed = MathUtil.clamp(3000 + (range * 200), 3000, 3900);
        return speed;
    }

    // ===== TILT Methods  ===================================

    public void setTiltAngle(double angle){
        tiltAngleSetPoint = MathUtil.clamp(angle, TiltConstants.minEncoderPosition, TiltConstants.maxEncoderPosition);
        tiltControl.setSetpoint(tiltAngleSetPoint);
    }

    public boolean tiltInPosition() {
        boolean inPosition = (Math.abs(tiltAngleSetPoint - currentTiltAngle) < TiltConstants.tiltThresholdDeg);
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
    public void runTiltPID() {
        double output = tiltControl.calculate(currentTiltAngle);

        // clip output to acceptable range
        output = MathUtil.clamp(output, TiltConstants.kMinOutput, TiltConstants.kMaxOutput);

        // if we are lowering, and are close to our target, just set power to zero to brake
        if ((output < 0) && (Math.abs(tiltControl.getPositionError()) < 5)) {
            output = 0;
        }

        // Send power to motors.  Flip the power for the left side.
        tiltPower = output;
        tiltLeft.set(-tiltPower);
        tiltRight.set(tiltPower);

        SmartDashboard.putNumber("Tilt Out", output);
    }

    // ===== SHOOTER Methods  ===============================

    public void setShooterRPM(double speed){
        shooterSpeedSetPoint = speed;
        shooterBot.setRPM(shooterSpeedSetPoint);
        shooterTop.setRPM(shooterSpeedSetPoint);
        if (speed > 0) {
            Globals.lastShooterSpeed = shooterSpeedSetPoint;            
        }
    }
    
    public void stopShooter(){
        setShooterRPM(0);
    }

    public boolean shooterUpToSpeed() {
        return (shooterSpeedSetPoint > 0) && (Math.abs(shooterSpeedSetPoint - shooterSpeedBot) < ShooterConstants.speedThresholdRPM);
    }
    
    public boolean readyToShoot() {
        return noteInIntake() && tiltInPosition() && shooterUpToSpeed();
    }


    // ===== INTAKE Methods ==================

    public double getNoteSensorValue(){
        return rangeFinder.getVoltage();
    }

    public boolean noteInIntake(){
        return (noteSensor > BatonConstants.seeingNote);
    }

    public void collect (){
        intake.set(BatonConstants.collect);
        setState(BatonState.COLLECTING);
    }

    public void fire (){
        if (shooterUpToSpeed()){
            intake.set(BatonConstants.fire);
            setState(BatonState.SHOOTING);
            driver.setRumble(RumbleType.kLeftRumble, 0);
        }  else {
            driver.setRumble(RumbleType.kLeftRumble, 1);
        }
    }

    public void stopIntake (){
       driver.setRumble(RumbleType.kLeftRumble, 0);
       intake.set(BatonConstants.stopCollector);
    }

    public void eject (){
        intake.set(BatonConstants.eject);
    }

    // ============ Public Command Interface  ========================================
    public Command collectCmd()                     {return this.runOnce(() -> collect());}
    public Command ejectCmd()                       {return runOnce(() -> eject());}
    public Command fireCmd()                        {return runOnce(() -> fire());}
    public Command setShooterRPMCmd(double speed)   {return runOnce(() -> setShooterRPM(speed));}
    public Command setTiltAngleCmd(double angle)    {return runOnce(() -> setTiltAngle(angle));}
    public Command stopIntakeCmd()                  {return runOnce(() -> stopIntake());}
    
}
