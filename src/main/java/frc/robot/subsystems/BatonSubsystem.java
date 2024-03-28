package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BatonConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TiltConstants;
import frc.robot.utils.BackImageSource;
import frc.robot.utils.BatonState;
import frc.robot.utils.FLEXShooter;
import frc.robot.utils.FrontImageSource;
import frc.robot.utils.Globals;
import frc.robot.utils.LEDmode;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkAnalogSensor;

public class BatonSubsystem extends SubsystemBase {
    private CANSparkMax intake;
    private CANSparkMax tiltLeft;
    private CANSparkMax tiltRight;
    private FLEXShooter shooterTop;
    private FLEXShooter shooterBot;
    private ProfiledPIDController  tiltControl;
    private AbsoluteEncoder tiltEncoder;
    private RelativeEncoder tiltEncoderRel;
    private Timer           stateTimer = new Timer();

    private boolean tiltInPosition;

    private boolean shooterUpToSpeed;
    private boolean topUpToSpeed;
    private boolean botUpToSpeed;    
    private boolean rememberToStopIntake;
    
    private double tiltAngleSetPoint;
    private double currentTiltAngle;
    private double tiltPower;
    private double shooterSpeedSetPoint;
    private double shooterSpeedTop;
    private double shooterSpeedBot;
    private double noteSensorIntake;
    private double noteSensorShooter;
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
        intake.restoreFactoryDefaults();
        rangeFinder = intake.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
        intake.burnFlash();
        
        tiltLeft  = new CANSparkMax(BatonConstants.tiltLeftID, MotorType.kBrushless);
        tiltLeft.restoreFactoryDefaults();
        tiltLeft.setIdleMode(TiltConstants.kMotorIdleMode);
        tiltLeft.setSmartCurrentLimit(TiltConstants.kMotorCurrentLimit);
//        tiltLeft.setSoftLimit(SoftLimitDirection.kForward, TiltConstants.softLimitRev);
//        tiltLeft.setSoftLimit(SoftLimitDirection.kReverse, -TiltConstants.softLimitRev);
//        tiltLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
//        tiltLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);
        tiltLeft.burnFlash();

        tiltEncoderRel = tiltLeft.getEncoder();
    
        tiltRight = new CANSparkMax(BatonConstants.tiltRightID, MotorType.kBrushless);
        tiltRight.restoreFactoryDefaults();
        tiltRight.setIdleMode(TiltConstants.kMotorIdleMode);
        tiltRight.setSmartCurrentLimit(TiltConstants.kMotorCurrentLimit);
//        tiltRight.setSoftLimit(SoftLimitDirection.kForward, TiltConstants.softLimitRev);
//        tiltRight.setSoftLimit(SoftLimitDirection.kReverse, -TiltConstants.softLimitRev);
//       tiltRight.enableSoftLimit(SoftLimitDirection.kForward, true);
//        tiltRight.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
        tiltEncoder = tiltRight.getAbsoluteEncoder(Type.kDutyCycle);
        tiltEncoder.setPositionConversionFactor(TiltConstants.kEncoderPositionFactor);
        tiltEncoder.setVelocityConversionFactor(TiltConstants.kEncoderVelocityFactor);
        tiltRight.burnFlash();

/*
            headingLockController = new ProfiledPIDController(AutoConstants.kPHeadingLockController, 
                                                      AutoConstants.kIHeadingLockController, 
                                                      AutoConstants.kDHeadingLockController, 
                                                      AutoConstants.kHeadingLockConstraints );
    headingLockController.enableContinuousInput(-Math.PI, Math.PI);
    headingLockController.setTolerance(AutoConstants.kDHeadingLockTollerance);
    */
  
        tiltControl = new ProfiledPIDController(TiltConstants.kP, TiltConstants.kI, TiltConstants.kD, TiltConstants.kConstraints);    
        tiltControl.setTolerance(TiltConstants.kTollerance);
        //tiltControl.setIZone(TiltConstants.kZone);
        //tiltControl.setIntegratorRange(TiltConstants.kIMin,TiltConstants.kIMax);
        tiltControl.setGoal(0);

        shooterBot = new FLEXShooter("Bot", BatonConstants.shooterBotID, true);
        shooterTop = new FLEXShooter("Top", BatonConstants.shooterTopID, false);

        init();
    }

    public void init(){
        manualTiltAngle    = BatonConstants.defaultTilt;
        manualShooterSpeed = BatonConstants.defaultRPM;
        manualShooting = false;
        rememberToStopIntake = true;

        setState(BatonState.IDLE);
        setShooterRPM(0);
        setTiltAngle(0);
        intake.set(BatonConstants.stopCollector);
        Globals.setNoteTracking(false);
        Globals.setSpeakerTracking(false);
        stateTimer.restart();
        tiltControl.reset(getSafeTiltAngle());
    }

    @Override
    public void periodic() { 

        // Read baton sensors
        currentTiltAngle    = getSafeTiltAngle(); 
        Globals.batonIsDown = (currentTiltAngle < 1.0);
       
        // noteSensor          = getNoteSensorValue();
        noteSensorShooter   = shooterBot.getVoltage();
        noteSensorIntake    = shooterTop.getVoltage();

        tiltInPosition      = calculateTiltInPosition();
        shooterUpToSpeed    = areShootersUpToSpeed();

        Globals.noteInIntake = noteInIntake();

        // control the baton angle and shooter speed.
        if (Globals.getSpeakerTracking()) {
             double range = 0;

            // Deterine which target location method we should use.
            if (Globals.speakerTarget.valid) {
                range = Globals.speakerTarget.range;
            } else if (Globals.odoTarget.valid) {
                range = Globals.odoTarget.range;
            }

            // Prep Baton if we can see the target, or if ODO has us within valid range.
            if (Globals.speakerTarget.valid || 
                ((range > ShooterConstants.MinTargetRange) && (range < ShooterConstants.MaxTargetRange))) {
                setTiltAngle(rangeToAngle(range)); 
                setShooterRPM(rangeToRPM(range));
            } 
            manualShooting = false;

        } else if(Globals.getAmplifying()){
            //being done in state machine (so nothing)
            manualShooting = false;

        } else if (manualShooting) {
            setTiltAngle(manualTiltAngle);
            setShooterRPM(manualShooterSpeed);

        } //else {   // I'd ike to remove these so I can control the shooter from Auto.
//            setTiltAngle(0);
//            setShooterRPM(0);
//        }
     
        runTiltPID();
        runStateMachine();

        SmartDashboard.putString("Intake Range",        String.format("%.2f", shooterTop.getVoltage()));

        SmartDashboard.putNumber("tilt setpoint",       tiltAngleSetPoint);
        SmartDashboard.putString("tilt angle",          String.format("%.2f",currentTiltAngle));
        SmartDashboard.putNumber("tilt Power",              tiltRight.getAppliedOutput());
        SmartDashboard.putBoolean("tilt In Position",       tiltIsInPosition());
        SmartDashboard.putBoolean("Note In Intake",         noteInIntake());
        SmartDashboard.putNumber("tilt relative encoder",   tiltEncoderRel.getPosition());
        SmartDashboard.putNumber("tilt Error",              tiltAngleSetPoint - currentTiltAngle);

        SmartDashboard.putNumber("shooter setpoint", shooterSpeedSetPoint);
        SmartDashboard.putNumber("shooter bot RPM", shooterSpeedBot);
        SmartDashboard.putNumber("shooter top RPM", shooterSpeedTop);
        SmartDashboard.putBoolean("top UTS",        topUpToSpeed);
        SmartDashboard.putBoolean("bot UTS",        botUpToSpeed);
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
                
                if ((tiltAngleSetPoint == TiltConstants.homeAngle) && !tiltIsInPosition()) {
                    // Warn if batton in not fully lowered
                    Globals.setLEDMode(LEDmode.LOWERING);

                    // Assist Baton lowering by spinning wheels backwards as it passes the Bumper & Frame
                    if ((currentTiltAngle < 20) && (currentTiltAngle > 1.5)) {
                        intake.set(BatonConstants.eject);
                        rememberToStopIntake = true;
                    }
            
                } else {
                    if (!Globals.getSpeakerTracking()) {
                        if (DriverStation.isTeleopEnabled()) {
                            Globals.setLEDMode(LEDmode.SPEEDOMETER);
                        }
                    }

                    if (rememberToStopIntake) {
                        intake.set(BatonConstants.stopCollector);   
                        rememberToStopIntake = false; 
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
                } else if(stateTimer.hasElapsed(2.5)){
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

            case AMP_APPROACH:
                if (Globals.ampTarget.valid && (Globals.ampTarget.range > 0.01) && (Globals.ampTarget.range < 0.1)) {
                    setTiltAngle(TiltConstants.ampLowAngle);
                    setState(BatonState.AMP_TILTING);
                }
                break;

            case AMP_READY:
                break;

            case AMP_TILTING:
                if (tiltIsInPosition()){
                    eject();
                    setState(BatonState.AMP_EJECTING);
                }
                break;
            
            case AMP_EJECTING:
                if (!noteInIntake()){
                    setState(BatonState.AMP_HOLDING);
                }
                break;

            case AMP_HOLDING:
                if(stateTimer.hasElapsed(0.25)){
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
                    VisionSubsystem.setFrontImageSource(FrontImageSource.NOTE);
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

    public  void setSpeakerTracking(boolean on){
        Globals.setSpeakerTracking(on);
    }

    public  void setNoteTracking(boolean on){
        Globals.setNoteTracking(on);
    }

    // ===== TILT Methods  ===================================
    public double rangeToAngle(double range) {
        double X2 =   -3.2043;  // Was -3.204
        double X  =   29.8560;  // Was 29.85
        double C  =  -35.0   ;  // Was -33.96


        double angle = (X2 * range * range) + (X * range) + C;
        return angle;
    }
        
    public boolean tiltIsInPosition() {
        return tiltInPosition;
    }

    public void setTiltAngle(double angle){
        // load new setpoint and reset "inPosition" if it has changed
        double newSetpoint = MathUtil.clamp(angle, TiltConstants.minEncoderPosition, TiltConstants.maxEncoderPosition);
        if (newSetpoint != tiltAngleSetPoint) {
            tiltAngleSetPoint = newSetpoint;
            tiltControl.setGoal(tiltAngleSetPoint);
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

        /*
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
        */

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
        return  tiltIsInPosition() && shooterIsUpToSpeed() && Globals.robotAtHeading;
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
        return ((noteSensorIntake > BatonConstants.seeingNote) && (noteSensorIntake < BatonConstants.readingError) );
    }

    public void collect (){
        VisionSubsystem.setFrontImageSource(FrontImageSource.NOTE);
        intake.set(BatonConstants.collect);
        setState(BatonState.COLLECTING);
        Globals.setLEDMode(LEDmode.NOTE_COLLECTING);
    }

    public void fire (){
        // could be shooting to speaker or into Amp
        if (Globals.getAmplifying()) {
            setState(BatonState.AMP_TILTING);
        } else {
            VisionSubsystem.setBackImageSource(BackImageSource.SPEAKER);
            if (shooterIsUpToSpeed()){
                intake.set(BatonConstants.fire);
                setState(BatonState.SHOOTING);
            }
        }
    }

    // Kist fire the ring with whatever speed is set.
    public void lob (){
        intake.set(BatonConstants.fire);
        setState(BatonState.SHOOTING);
    }

    public void autoAmplify (boolean enable){
        if (enable) {
            VisionSubsystem.setFrontImageSource(FrontImageSource.AMP);
            if (currentState == BatonState.HOLDING){
                Globals.setAmplifying(true);
                setTiltAngle(TiltConstants.ampTrackAngle);
                setState(BatonState.AMP_APPROACH);
            }
        } else {
            VisionSubsystem.setFrontImageSource(FrontImageSource.NOTE);
            Globals.setAmplifying(false);
            setTiltAngle(TiltConstants.homeAngle);
            intake.set(BatonConstants.stopCollector);
            setState(BatonState.IDLE);
        }
    }

    public void manualAmplify (boolean enable){
        if (enable) {
            if (currentState == BatonState.HOLDING){
                Globals.setAmplifying(true);
                setTiltAngle(TiltConstants.ampLowAngle);
                setState(BatonState.AMP_READY);
            }
        } else {
            Globals.setAmplifying(false);
            setTiltAngle(TiltConstants.homeAngle);
            intake.set(BatonConstants.stopCollector);
            setState(BatonState.IDLE);
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
        setManualTiltAngle(manualTiltAngle + bump);
    }

    public void bumpShooter(double bump) {
        setManualShooterSpeed(manualShooterSpeed + bump);
    }

    public void setManualShooting(boolean on){
        manualShooting = on;
    }

    public void setManualTiltAngle(double angle) {
        manualTiltAngle = MathUtil.clamp(angle, 0, BatonConstants.maxTiltAngle);
    }

    public void setManualShooterSpeed(double speedRPM) {
        manualShooterSpeed = MathUtil.clamp(speedRPM, 0, BatonConstants.maxShooterRPM);
    }

    public void setSpeedAndTilt(double speed, double angle){
        manualShooterSpeed = MathUtil.clamp(speed, 0, BatonConstants.maxShooterRPM);
        manualTiltAngle = MathUtil.clamp(angle, 0, BatonConstants.maxTiltAngle);
    }
  
}
