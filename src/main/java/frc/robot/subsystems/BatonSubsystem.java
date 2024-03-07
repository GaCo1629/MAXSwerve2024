package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BatonConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TiltConstants;
import frc.robot.utils.BatonState;
import frc.robot.utils.FLEXShooter;
import frc.robot.utils.GPIDController;
import frc.robot.utils.Globals;
import frc.robot.utils.LEDmode;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

public class BatonSubsystem extends SubsystemBase {
    private CANSparkMax intake;
    private CANSparkMax tiltLeft;
    private CANSparkMax tiltRight;
    private FLEXShooter shooterTop;
    private FLEXShooter shooterBot;
    private GPIDController tiltControl;
    private AbsoluteEncoder tiltEncoder;
    private RelativeEncoder tiltEncoderRel;
    private Timer       stateTimer = new Timer();

    private boolean tiltInPosition;

    private boolean shooterUpToSpeed;
    private boolean topUpToSpeed;
    private boolean botUpToSpeed;    
    
    private double tiltAngleSetPoint;
    private double currentTiltAngle;
    private double tiltPower;
    private double shooterSpeedSetPoint;
    private double shooterSpeedTop;
    private double shooterSpeedBot;
    private double noteSensor;
    private BatonState currentState;

    private boolean manualShooting;
    private double manualTiltAngle;
    private double manualShooterSpeed;

    private final SparkAnalogSensor   rangeFinder; 

    //private PS4Controller driver;
    //private Joystick copilot_1;
    //private Joystick copilot_2;

    public BatonSubsystem (PS4Controller driver, Joystick copilot_1, Joystick copilot_2){
        //this.driver = driver;
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
        tiltLeft.setSoftLimit(SoftLimitDirection.kForward, 0);
        tiltLeft.setSoftLimit(SoftLimitDirection.kReverse, -TiltConstants.softLimitRev);
        tiltLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
        tiltLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);
        tiltLeft.burnFlash();

        tiltEncoderRel = tiltLeft.getEncoder();
    
        tiltRight = new CANSparkMax(BatonConstants.tiltRightID, MotorType.kBrushless);
        tiltRight.restoreFactoryDefaults();
        tiltRight.setIdleMode(TiltConstants.kMotorIdleMode);
        tiltRight.setSmartCurrentLimit(TiltConstants.kMotorCurrentLimit);
        tiltRight.setSoftLimit(SoftLimitDirection.kForward, TiltConstants.softLimitRev);
        tiltRight.setSoftLimit(SoftLimitDirection.kReverse, 0);
        tiltRight.enableSoftLimit(SoftLimitDirection.kForward, true);
        tiltRight.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
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
        manualTiltAngle    = BatonConstants.defaultTilt;
        manualShooterSpeed = BatonConstants.defaultRPM;
        manualShooting = false;

        setState(BatonState.IDLE);
        setShooterRPM(0);
        setTiltAngle(0);
        intake.set(BatonConstants.stopCollector);
        Globals.setNoteTracking(false);
        Globals.setSpeakerTracking(false);
        stateTimer.restart();
    }

    @Override
    public void periodic() { 

        // Read baton sensors
        currentTiltAngle    = getSafeTiltAngle(); 
        
        noteSensor          = getNoteSensorValue();
        tiltInPosition      = calculateTiltInPosition();
        shooterUpToSpeed    = areShootersUpToSpeed();

        // control the baton angle and shooter speed.
        if (Globals.getSpeakerTracking() && (Globals.speakerTarget.valid)) {
            setTiltAngle(rangeToAngle(Globals.speakerTarget.range)); 
            setShooterRPM(rangeToRPM(Globals.speakerTarget.range));
        } else if(Globals.getAmplifying()){
            //being done in state machine (so nothing)

        } else {
            if (manualShooting) {
                setTiltAngle(manualTiltAngle);
                setShooterRPM(manualShooterSpeed);
            } else{
                setTiltAngle(0);
                setShooterRPM(0);
            }
        }

        runTiltPID();
        runStateMachine();

        SmartDashboard.putNumber("Intake Range",    rangeFinder.getVoltage());

        SmartDashboard.putNumber("tilt setpoint",   tiltAngleSetPoint);
        SmartDashboard.putNumber("tilt angle",      currentTiltAngle);
        SmartDashboard.putNumber("tilt Power",      tiltRight.getAppliedOutput());
        SmartDashboard.putBoolean("tilt In Position",tiltIsInPosition());
        SmartDashboard.putBoolean("Note In Intake", noteInIntake());
        SmartDashboard.putNumber("tilt relative encoder", tiltEncoderRel.getPosition());
        SmartDashboard.putNumber("tilt Error", tiltAngleSetPoint - currentTiltAngle);

        SmartDashboard.putNumber("shooter setpoint", shooterSpeedSetPoint);
        SmartDashboard.putNumber("shooter bot RPM", shooterSpeedBot);
        SmartDashboard.putNumber("shooter top RPM", shooterSpeedTop);
        SmartDashboard.putBoolean("top UTS", topUpToSpeed);
        SmartDashboard.putBoolean("bot UTS", botUpToSpeed);
        SmartDashboard.putBoolean("Ready To Shoot", readyToShoot());
    
        SmartDashboard.putString("BatonState",      currentState.toString());

        SmartDashboard.putBoolean("Manual Shooting",manualShooting);
        SmartDashboard.putNumber("Manual Tilt",     manualTiltAngle);
        SmartDashboard.putNumber("Manual Speed",    manualShooterSpeed);
    }

    /**
     * Baton state machine.  Manages the sequencing of intake, aim and fire.
     */
    public void runStateMachine(){
        switch (currentState) {
            case IDLE:
                if (noteInIntake()) {
                    setState(BatonState.HOLDING);
                }

                // Warn if batton in not fully lowered
                if ((tiltAngleSetPoint == TiltConstants.homeAngle) && !tiltIsInPosition()) {
                    Globals.setLEDMode(LEDmode.LOWERING);
                } else {
                    if (!Globals.getSpeakerTracking()) {
                        Globals.setLEDMode(LEDmode.SPEEDOMETER);
                    }
                }
                // Exits by button press.
                break;

            case COLLECTING:
                if (noteInIntake()){
                    stopIntake();
                    setState(BatonState.HOLDING);
                    Globals.setNoteTracking(false);
                } else {
                    if (Globals.noteTarget.valid) {
                        Globals.setLEDMode(LEDmode.NOTE_DETECTED);
                    } else {
                        Globals.setLEDMode(LEDmode.NOTE_COLLECTING);                        
                    }
                }
                break;
        
            case HOLDING:
                if (Globals.speakerTarget.valid){
                    Globals.setLEDMode(LEDmode.SPEAKER_DETECTED);
                } else {
                     Globals.setLEDMode(LEDmode.NOTE_HOLDING);
                }
                // Exits by "fire" button press.
                break;
                
            case AUTO_SHOOT:
                // Exits by being on target and firing
                if (readyToShoot()){
                   intake.set(BatonConstants.fire);
                   setState(BatonState.SHOOTING); 
                } else if(stateTimer.hasElapsed(2)){
                   intake.set(BatonConstants.fire);
                   Globals.setLEDMode(LEDmode.SHOOTING_TIMEOUT);
                   setState(BatonState.SHOOTING); 
                }
                break;
                
            case SHOOTING:
                Globals.setLEDMode(LEDmode.SHOOTING);
                if (!noteInIntake()){
                    setState(BatonState.SHOOTING_WAIT);
                }
                break;

            case SHOOTING_WAIT:    
                if (stateTimer.hasElapsed(0.25)){
                    stopIntake();
                    stopShooter();
                    Globals.setSpeakerTracking(false);
                    setState(BatonState.IDLE);
                }
                 break;

            case TILTING:
                if (tiltIsInPosition()){
                    eject();
                    setState(BatonState.EJECTING);
                }
                break;
            
            case EJECTING:
                if (!noteInIntake()){
                    setTiltAngle(TiltConstants.ampHighAngle);
                    setState(BatonState.AMP_SCORING);
                }
                break;

            case AMP_SCORING:
                if (tiltIsInPosition()){
                    setState(BatonState.AMP_WAIT);
                }
                break;

            case AMP_WAIT:
                if (stateTimer.hasElapsed(0.5)){
                    stopIntake();
                    setTiltAngle(TiltConstants.homeAngle);
                    Globals.setAmplifying(false);
                    setState(BatonState.IDLE);
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
        double angle = (-3.558 * range * range) + (31.335 * range) - 30.389;
        //double angle = (-3.025 * range * range) + (28.00 * range) - 26.311;
        return angle - 6;
    }
    
    public  void setSpeakerTracking(boolean on){
        Globals.setSpeakerTracking(on);
    }

    public  void setNoteTracking(boolean on){
        Globals.setNoteTracking(on);
    }

    // ===== TILT Methods  ===================================

    public boolean tiltIsInPosition() {
        return tiltInPosition;
    }

    public void setTiltAngle(double angle){
        // load new setpoint and reset "inPosition" if it has changed
        double newSetpoint = MathUtil.clamp(angle, TiltConstants.minEncoderPosition, TiltConstants.maxEncoderPosition);
        if (newSetpoint != tiltAngleSetPoint) {
            tiltAngleSetPoint = newSetpoint;
            tiltControl.setSetpoint(tiltAngleSetPoint);
            tiltInPosition = calculateTiltInPosition();
        }
    }


    public double getSafeTiltAngle() {
        double safe = tiltEncoder.getPosition();
        if (safe > 180) safe = 0;

        return safe;
    }

    public boolean calculateTiltInPosition(){
        return (Math.abs(tiltAngleSetPoint - currentTiltAngle) < TiltConstants.tiltThresholdDeg);
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

        // if we are in auto, then make sure we get to the ground quickly
        if (tiltAngleSetPoint == 0){
            if (currentTiltAngle > 6) {
                output = -0.2;
            } else  {
                output = 0;
            }
        } else 
        // if we are lowering, and are close to our target, just set power to zero to brake
        if ((output < 0) && (Math.abs(tiltControl.getPositionError()) < 2)) {
            output = 0;
        }

        // Send power to motors.  Flip the power for the left side.
        tiltPower = output;
        tiltLeft.set(-tiltPower);
        tiltRight.set(tiltPower);

        SmartDashboard.putNumber("Tilt Out", output);
    }

    // ===== SHOOTER Methods  ===============================
    public boolean shooterIsUpToSpeed() {
        return shooterUpToSpeed;
    }

    public void setShooterRPM(double speed){
        if (speed != shooterSpeedSetPoint) {
            shooterSpeedSetPoint = speed;
            shooterBot.setRPM(shooterSpeedSetPoint);
            shooterTop.setRPM(shooterSpeedSetPoint);
            if (speed > 0) {
                Globals.lastShooterSpeed = shooterSpeedSetPoint;      
                //  shooterUpToSpeed = false;  //   maybe do this... 
            }
        }
    }

    public double rangeToRPM(double range) {
        double speed = MathUtil.clamp(ShooterConstants.baseShooterSpeed + (range * 200), 
                                      ShooterConstants.baseShooterSpeed, 
                                      ShooterConstants.topShooterSpeed);
        return speed;
    }
    
    public void stopShooter(){
        setShooterRPM(0);
    }
       
    public boolean readyToShoot() {
        // return noteInIntake() && tiltIsInPosition() && shooterIsUpToSpeed();
        return tiltIsInPosition() && shooterIsUpToSpeed();
    }
     
    public boolean areShootersUpToSpeed() {
        shooterSpeedTop = shooterTop.getRPM();
        shooterSpeedBot = shooterBot.getRPM();

        topUpToSpeed = isShooterUpToSpeed(shooterBot.getRPM());
        botUpToSpeed = isShooterUpToSpeed(shooterTop.getRPM());
        
        return ((shooterSpeedSetPoint > 0) && (topUpToSpeed || botUpToSpeed));
    }

    public boolean isShooterUpToSpeed(double liveSpeed) {
        return (Math.abs(shooterSpeedSetPoint - liveSpeed) < ShooterConstants.speedThresholdRPM);
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
        Globals.setLEDMode(LEDmode.NOTE_COLLECTING);
    }

    public void fire (){
       SmartDashboard.putBoolean("Shooter Up To Speed",shooterIsUpToSpeed());
       SmartDashboard.putNumber("Shooter Up To Speed test",stateTimer.get());
        if (shooterIsUpToSpeed()){
            intake.set(BatonConstants.fire);
            setState(BatonState.SHOOTING);
       }
    }

    public void amplify (){
        if (currentState == BatonState.HOLDING){
            Globals.setAmplifying(true);
            setTiltAngle(TiltConstants.ampLowAngle);
            setState(BatonState.TILTING);
        }
    }

    public void stopIntake (){
        intake.set(BatonConstants.stopCollector);
        Globals.setNoteTracking(false);
    }

    public void eject (){
        intake.set(BatonConstants.eject);
    }

    //  =========  Manual Shooting Commands
    public void bumpTilt(double bump) {
        manualTiltAngle += bump;
        manualTiltAngle = MathUtil.clamp(manualTiltAngle, 0, BatonConstants.maxTiltAngle);
    }

    public void bumpShooter(double bump) {
        manualShooterSpeed += bump;
        manualShooterSpeed = MathUtil.clamp(manualShooterSpeed, 0, BatonConstants.maxShooterRPM);
    }

    public void enableManualShooting(boolean on){
        manualShooting = on;
    }

    // ============ Public Command Interface  ========================================
    public Command collectCmd()                     {return runOnce(() -> collect());}
    public Command ejectCmd()                       {return runOnce(() -> eject());}
    public Command fireCmd()                        {return run(() -> fire());}
    public Command amplifyCmd()                     {return runOnce(() -> amplify());}
    public Command setShooterRPMCmd(double speed)   {return runOnce(() -> setShooterRPM(speed));}
    public Command setTiltAngleCmd(double angle)    {return runOnce(() -> setTiltAngle(angle));}
    public Command stopIntakeCmd()                  {return runOnce(() -> stopIntake());}
    
    public Command bumpTiltCmd(double bump)         {return runOnce(() -> bumpTilt(bump));}
    public Command bumpShooterCmd(double bump)      {return runOnce(() -> bumpShooter(bump));}
    public Command enableManualShootingCmd(boolean on) {return runOnce(() -> enableManualShooting(on));}
    public Command setNoteTrackingCmd(boolean on)   {return runOnce(() -> setNoteTracking(on));}
    public Command setSpeakerTrackingCmd(boolean on){return runOnce(() -> setSpeakerTracking(on));}

    
}
