package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.utils.Globals;

public class LiftSubsystem extends SubsystemBase{
    private CANSparkFlex rightLift;
    private RelativeEncoder rightEncoder;;

    private double robotAngle       = 0;
    private double liftTurns  = 0;
    
    //private PS4Controller driver;
    private PS4Controller  copilot_1;
    //private Joystick copilot_2;


    public LiftSubsystem (PS4Controller driver, PS4Controller  copilot_1, Joystick copilot_2){
        //this.driver = driver;
        this.copilot_1 = copilot_1;
        //this.copilot_2 = copilot_2;

        if (Globals.enableLIftSubsystem) {
            rightLift = new CANSparkFlex(LiftConstants.rightLiftID, MotorType.kBrushless);
            rightLift.restoreFactoryDefaults();
            rightEncoder = rightLift.getEncoder();
            rightLift.burnFlash();
        }

        rightEncoder.setPosition(0);
        Globals.liftIsEnabled = false;
    }

    public void init() {
        resetHome();
        Globals.liftIsEnabled = false;
    }

    @Override
    public void periodic(){
        robotAngle = Globals.robotRoll;
        double power = 0;

        // do nothing if susbstem is not present
        if (Globals.enableLIftSubsystem) {
            liftTurns = rightEncoder.getPosition();
        
            if (Globals.liftIsEnabled) {
                power = -copilot_1.getRightY();  // forward (-ve) Stick up makes the lifter go up.

                if ((liftTurns < 0) && (power < 0)) {
                    power = MathUtil.clamp(power, -0.15, 1) ;
                } else if ((liftTurns > 108) && (power > 0)) {  // updated for 20:1 gearbox
                    power = MathUtil.clamp(power, -1, 0);
                }
            } 
            rightLift.set(power); 
        }   

        Globals.liftSubsystemFaults = getFaults();
        SmartDashboard.putBoolean("Lift Fault", Globals.liftSubsystemFaults != 0);
        SmartDashboard.putString("Lift Faults", String.format("%s", Integer.toBinaryString(Globals.liftSubsystemFaults)));

        SmartDashboard.putBoolean("Lift Enable", Globals.liftIsEnabled);
        SmartDashboard.putNumber("Lift Turns", liftTurns);
    }

    public void resetHome() {
        rightEncoder.setPosition(0);
        Globals.liftIsEnabled = false;
    }

    public void enableLiftSystem() {
        Globals.liftIsEnabled = true;
    }

    public int getFaults() {
        return rightLift.getFaults();
    }
}

