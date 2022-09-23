package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Calibration implements Sendable {
    private double m_inputA;
    private double m_inputB;
    private double m_inputC;

    private double m_outputA;
    private double m_outputB;
    private double m_outputC;

    private double m_calibA;
    private double m_calibB;
    private double m_calibC;

    public Calibration(double inputA, double inputB, double inputC, double outputA, double outputB, double outputC){
        setValues(inputA, inputB, inputC, outputA, outputB, outputC);
    }

    public double getCalibratedValue(double input){
        return m_calibA * input * input + m_calibB * input + m_calibC;
    }

    public void computeCoefficients(){
        m_calibA =   m_outputA / ((m_inputA - m_inputB) * (m_inputA - m_inputC)) - m_outputB / ((m_inputA - m_inputB) * (m_inputB - m_inputC)) + m_outputC / ((m_inputA - m_inputC) * (m_inputB - m_inputC));
        m_calibB = - m_outputA * (m_inputB + m_inputC) / ((m_inputA - m_inputB) * (m_inputA - m_inputC)) + m_outputB * (m_inputA + m_inputC) / ((m_inputA - m_inputB) * (m_inputB - m_inputC)) + m_outputC * (m_inputA + m_inputB)/ ((m_inputA - m_inputC) * (m_inputC - m_inputB));
        m_calibC =   m_outputA * m_inputB * m_inputC / ((m_inputA - m_inputB) * (m_inputA - m_inputC)) - m_outputB * m_inputA * m_inputC / ((m_inputA - m_inputB) * (m_inputB - m_inputC)) + m_outputC * m_inputA * m_inputB / ((m_inputA - m_inputC) * (m_inputB - m_inputC));
    }

    public void setValues(double inputA, double inputB, double inputC, double outputA, double outputB, double outputC){
        m_inputA = inputA;
        m_inputB = inputB;
        m_inputC = inputC;

        m_outputA = outputA;
        m_outputB = outputB;
        m_outputC = outputC;
        computeCoefficients();
    }

    public double getInputA(){
        return m_inputA;
    }

    public void setInputA(double inputA) {
        m_inputA = inputA;
    }

    public double getInputB(){
        return m_inputB;
    }

    public void setInputB(double inputB) {
        m_inputB = inputB;
    }

    public double getInputC(){
        return m_inputC;
    }

    public void setInputC(double inputC){
        m_inputC = inputC;
    }

    public double getOutputA(){
        return m_outputA;
    }

    public void setOutputA(double outputA){
        m_outputA = outputA;
    }

    public double getOutputB(){
        return m_outputB;
    }

    public void setOutputB(double outputB){
        m_outputB = outputB;
    }

    public double getOutputC() {
        return m_outputC;
    }

    public void setOutputC(double outputC){
        m_outputC = outputC;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");

        builder.addDoubleProperty("Input A", this::getInputA, this::setInputA);
        builder.addDoubleProperty("Input B", this::getInputB, this::setInputB);
        builder.addDoubleProperty("Input C", this::getInputC, this::setInputC);
        
        builder.addDoubleProperty("Output A", this::getOutputA, this::setOutputA);
        builder.addDoubleProperty("Output B", this::getOutputB, this::setOutputB);
        builder.addDoubleProperty("Output C", this::getOutputC, this::setOutputC);
    }
}
