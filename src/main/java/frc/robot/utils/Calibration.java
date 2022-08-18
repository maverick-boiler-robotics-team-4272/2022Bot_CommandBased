package frc.robot.utils;

public class Calibration {
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
}
