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
import com.revrobotics.CANSparkBase.SoftLimitDirection;

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
    //private PS4Controller copilot_1;
    //private Joystick copilot_2;

    public BatonSubsystem (PS4Controller driver, PS4Controller  copilot_1, Joystick copilot_2){
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
        tiltLeft.setSoftLimit(SoftLimitDirection.kForward, TiltConstants.softLimitRev);
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
        tiltRight.setSoftLimit(SoftLimitDirection.kReverse, -TiltConstants.softLimitRev);
        tiltRight.enableSoftLimit(SoftLimitDirection.kForward, true);
        tiltRight.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
        tiltEncoder = tiltRight.getAbsoluteEncoder(Type.kDutyCycle);
        tiltEncoder.setPositionConversionFactor(TiltConstants.kEncoderPositionFactor);
        tiltEncoder.setVelocityConversionFactor(TiltConstants.kEncoderVelocityFactor);
        tiltRight.burnFlash();

        tiltControl = new ProfiledPIDController(TiltConstants.kP, TiltConstants.kI, TiltConstants.kD, TiltConstants.kConstraints);    
        tiltControl.setIZone(TiltConstants.kIzone);
        tiltControl.setTolerance(TiltConstants.kTollerance);
        tiltControl.setGoal(0);

        shooterBot = new FLEXShooter("Bot", BatonConstants.shooterBotID, false);
        shooterTop = new FLEXShooter("Top", BatonConstants.shooterTopID, true);

        init();
    }

    public void init(){
        manualTiltAngle    = BatonConstants.defaultTilt;
        manualShooterSpeed = BatonConstants.defaultRPM;
        manualShooting = false;
        rememberToStopIntake = true;

        tiltLeft.getEncoder().setPosition(0);
        tiltRight.getEncoder().setPosition(0);

        setState(BatonState.IDLE);
        relaxBaton();
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
        Globals.noteAtShooter = noteAtShooter();

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

        }
     
        runTiltPID();
        runStateMachine();

        Globals.batonSubsystemFaults = getFaults();
        SmartDashboard.putBoolean("Baton Fault", Globals.batonSubsystemFaults != 0);
        SmartDashboard.putString("Baton Faults", String.format("%s", Integer.toBinaryString(Globals.batonSubsystemFaults)));

        SmartDashboard.putString("Intake Range",        String.format("%.2f", shooterTop.getVoltage()));

        SmartDashboard.putNumber("tilt setpoint",       tiltAngleSetPoint);
        SmartDashboard.putNumber("tilt angle",          currentTiltAngle);
        SmartDashboard.putNumber("tilt Power",              tiltRight.getAppliedOutput());
        SmartDashboard.putBoolean("tilt In Position",       tiltIsInPosition());
        SmartDashboard.putBoolean("Note In Intake",         noteInIntake());
        SmartDashboard.putBoolean("Note At Shooter",        noteAtShooter());
        
        SmartDashboard.putNumber("tilt relative encoder",   tiltEncoderRel.getPosition());
        SmartDashboard.putNumber("tilt Error",              tiltAngleSetPoint - currentTiltAngle);
        SmartDashboard.putNumber("Note Sensor Shooter",     noteSensorShooter);
        SmartDashboard.putNumber("Note Sensor Intake",      noteSensorIntake);

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
                    //if ((currentTiltAngle < 20) && (currentTiltAngle > 1.5)) {
                    //    intake.set(BatonConstants.eject);
                    //    rememberToStopIntake = true;
                    //}
            
                } else {
                    if (!Globals.getSpeakerTracking()) {
                        if (DriverStation.isTeleopEnabled()) {
                            Globals.setLEDMode(LEDmode.SPEEDOMETER);
                        }
                    }

                    if (rememberToStopIntake) {
                        intake.set(BatonConstants.stop);   
                        rememberToStopIntake = false; 
                    }
                }

                // Exits by button press.
                break;

            case COLLECTING:
                if (noteInIntake() || noteAtShooter()){
                    setState(BatonState.INDEXING);
                    Globals.setNoteTracking(false);
                } else {
                    if (Globals.noteTarget.valid) {
                        Globals.setLEDMode(LEDmode.NOTE_DETECTED);
                    } else {
                        Globals.setLEDMode(LEDmode.NOTE_COLLECTING);                        
                    }
                }
                break;

            case INDEXING:
                if (noteInIntake()){
                    setState(BatonState.HOLDING);
                }
                Globals.setLEDMode(LEDmode.NOTE_HOLDING);
                break;
        
            case HOLDING:
                if (Globals.speakerTarget.valid){
                    Globals.setLEDMode(LEDmode.SPEAKER_DETECTED);
                } else {
                    Globals.setLEDMode(LEDmode.NOTE_HOLDING);
                }

                // center note in intake
                if (!noteInIntake() && noteAtShooter()) {
                    intake.set(BatonConstants.indexDown);
                } else if (noteInIntake() && !noteAtShooter()){
                    intake.set(BatonConstants.indexUp);
                } else {
                    intake.set(BatonConstants.stop);
                }

                // check to see if the note had been ejected
                if (!noteInIntake() && !noteAtShooter()){
                    relaxBaton();
                    setState(BatonState.IDLE);
                }
                
                // Exits by "fire" button press.
                break;
            
            case AUTO_PRE_SHOOT:
                setState(BatonState.AUTO_SHOOT);   // guarentee at least one full periodic cycle before checking readyToShoot
                                                   // We do this to flush out any manual shooting prep position/speeds.
                break;
                
            case AUTO_SHOOT:
                // Exits by being on target and ready to fire
                if (readyToShoot()){
                   intake.set(BatonConstants.fire);
                   setState(BatonState.SHOOTING); 
                } else if(stateTimer.hasElapsed(1.5)){
                   intake.set(BatonConstants.fire);
                   Globals.setLEDMode(LEDmode.SHOOTING_TIMEOUT);
                   setState(BatonState.SHOOTING); 
                }
                break;
                
            case AUTO_SHOOT_FAST:
                // Exits by being up to speed
                if (shooterIsUpToSpeed()){
                   intake.set(BatonConstants.fire);
                   setState(BatonState.SHOOTING); 
                } else if(stateTimer.hasElapsed(1.5)){
                   intake.set(BatonConstants.fire);
                   Globals.setLEDMode(LEDmode.SHOOTING_TIMEOUT);
                   setState(BatonState.SHOOTING); 
                }
                break;
                
            case SHOOTING:
                Globals.setLEDMode(LEDmode.SHOOTING);
                if (!noteAtShooter() && !noteInIntake()){
                    setState(BatonState.SHOOTING_WAIT);
                }
                break;

            case SHOOTING_WAIT:    
                if (stateTimer.hasElapsed(0.0)){   // was 0.5
                    relaxBaton();
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
                    setTiltAngle(TiltConstants.ampHighAngle);
                    setState(BatonState.AMP_SCORING);
                }
                break;

            case AMP_SCORING:
                if (tiltIsInPosition()){
                    relaxBaton();
                    Globals.setAmplifying(false);
                    VisionSubsystem.setFrontImageSource(FrontImageSource.NOTE);
                    setState(BatonState.AMP_LOWERING);
                }
                break;

            case AMP_LOWERING:
                if (tiltIsInPosition() || stateTimer.hasElapsed(1)){
                    setState(BatonState.IDLE);
                }
                break;

            default:
                break;
        }
    }

    public void relaxBaton() {
        stopIntake();
        stopShooter();
        setTiltAngle(TiltConstants.homeAngle);
    }

    public void  setState(BatonState state) {
        currentState = state;
        Globals.batonState = state;
        stateTimer.restart();
    }

    public BatonState  getState() {
        return currentState;
    }

    // ===== TILT Methods  ===================================
    public double rangeToAngle(double range) {
        double X3 =     0.4245; 
        double X2 =    -7.0375; 
        double X  =   40.843;  
        double C  =  -43.196;  
        //   y = 0.4245x3 - 7.0375x2 + 40.843x - 43.196

        range = MathUtil.clamp(range, 1, 6.0);

        double angle = (X3 * range * range * range) + (X2 * range * range) + (X * range) + C;
        
        return MathUtil.clamp(angle, 0, 40);
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

    public void readyShooter (double range){
        SmartDashboard.putString("Mode", "Ready Shooter");
        double shooterSpeed = rangeToRPM (range);
        double angle = rangeToAngle(range);
        setShooterRPM(shooterSpeed);
        setTiltAngle(angle);
    }

    public double rangeToRPM(double range) {
        return MathUtil.clamp(2300 + (500 * range), 0,5000);
    }
    
    public void stopShooter(){
        setShooterRPM(0);
    }
       
    public boolean readyToShoot() {
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

    public boolean noteAtShooter(){
        return ((noteSensorShooter > BatonConstants.seeingNote) && (noteSensorShooter < BatonConstants.readingError) );
    }

    public void collect (){
        VisionSubsystem.setFrontImageSource(FrontImageSource.NOTE);
        intake.set(BatonConstants.collect);
        setState(BatonState.COLLECTING);
        Globals.setLEDMode(LEDmode.NOTE_COLLECTING);
    }

    public void fire (){
        // could be shooting to speaker or into Amp
        if (Globals.getAmplifying() && (currentState == BatonState.AMP_READY)) {
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
            relaxBaton();
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
            relaxBaton();
            setState(BatonState.IDLE);
        }
    }

    public void stopIntake (){
        intake.set(BatonConstants.stop);
        Globals.setNoteTracking(false);
    }

    public void creepIntake() {
        intake.set(BatonConstants.indexUp);    
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
        if (!manualShooting) {
            setTiltAngle(0);
            setShooterRPM(0);
        }
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

    public int getFaults() {
        return intake.getFaults() + tiltLeft.getFaults() + tiltRight.getFaults() + shooterBot.getFaults() + shooterTop.getFaults();
    }
  
}
