package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

public class Filter {
    double m_filteredValue = 0;
    double m_lastInputTime = 0;
    double m_elapsedTime = 0;

    double m_timeConstant = 1.0;
    boolean m_circular = false; 

    public Filter(double time_constant, boolean circular){
        m_timeConstant = time_constant;
        m_circular = circular;
        m_filteredValue = 0;
    }

    public Filter(double time_constant, boolean circular, double initialValue){
        m_timeConstant = time_constant;
        m_circular = circular;
        m_filteredValue = initialValue;
    }

    public void reset(){
        reset(0.0);
    }

    public void reset(double value){
        m_filteredValue = value;
        m_lastInputTime = Timer.getFPGATimestamp();
    }

    public double update(double value){
        double inputTime = Timer.getFPGATimestamp();
        m_elapsedTime =  inputTime - m_lastInputTime;
        double change = value - m_filteredValue;

        if (m_circular) {
            change = normalize(change);
        }

        m_filteredValue += change * m_timeConstant * m_elapsedTime;
        m_lastInputTime = inputTime;

        if (m_circular) {
            m_filteredValue = normalize(m_filteredValue);
        }

        return m_filteredValue;
    }

    private double normalize(double heading){
        while (heading > 180) {
            heading -= 360;
        }

        while (heading < -180) {
            heading += 180;
        }

        return heading;
    }
}
