package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class LiftSubsystem extends SubsystemBase{
    private CANSparkFlex leftLift = null;
    private CANSparkFlex rightLift = null;

    private double robotAngle       = 0;
    private double leftLiftSetpoint = 0;
    private double rightLiftSetpoint= 0;
    private double leftLiftHeight   = 0;
    private double rightLiftHeight  = 0;
    
    //private PS4Controller driver;
    //private Joystick copilot_1;
    //private Joystick copilot_2;


    public LiftSubsystem (PS4Controller driver, Joystick copilot_1, Joystick copilot_2){
        //this.driver = driver;
        //this.copilot_1 = copilot_1;
        //this.copilot_2 = copilot_2;

        if (Globals.enableLIftSubsystem) {
            leftLift = new CANSparkFlex(LiftConstants.leftLiftID, MotorType.kBrushless);
            rightLift = new CANSparkFlex(LiftConstants.rightLiftID, MotorType.kBrushless);
        }

        leftLiftSetpoint = 0;
        rightLiftSetpoint = 0;
    }

    @Override
    public void periodic(){
        robotAngle = Globals.roll;

        if (Globals.enableLIftSubsystem) {
            leftLiftHeight  = leftLift.getEncoder().getPosition();
            rightLiftHeight = rightLift.getEncoder().getPosition();
        }

        SmartDashboard.putNumber("Lift Setpoint Left",  leftLiftSetpoint);
        SmartDashboard.putNumber("Lift Height Left",  leftLiftHeight);
        SmartDashboard.putNumber("Lift Setpoint Right", rightLiftSetpoint);
        SmartDashboard.putNumber("Lift Height Right", rightLiftHeight);
        SmartDashboard.putNumber("Lift Robot Angle", robotAngle);
    }
}

