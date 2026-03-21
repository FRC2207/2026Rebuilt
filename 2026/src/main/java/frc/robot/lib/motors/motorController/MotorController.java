package frc.robot.lib.motors.motorController;

import org.littletonrobotics.junction.Logger;

//import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorController{
    private MotorControllerIO io;
    private final MotorControllerIOInputsAutoLogged inputs = new MotorControllerIOInputsAutoLogged();
    private String loggingKey;

    /** This is the central hub for all things motors! 
     * 
     * @param io         the io object for the motorController
     * @param loggingKey the path where inputs will be logged
     * 
     */
    public MotorController(MotorControllerIO io, String loggingKey) {
        this.io = io;
        this.loggingKey = loggingKey;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs(loggingKey, inputs);
    }

    /** Sets the motor percentage from -1 to 1. This is known as <i> duty cycle. </i>
     * @param percent this is a value from -1 to 1
     */
    public void setMotorPercent(double percent) {
        io.setMotorPercent(percent);
    }

    /** Sets the motor percentage from -12 to 12
     * @param voltage this is a value from -12 to 12
     */
    public void setMotorVoltage (double voltage) {
        io.setMotorVoltage(voltage);
    }

    /**
     * 
     * @return The applied <strong> voltage </strong> output of the duty cycle
     */
    public double getAppliedVolts(){
        return io.getAppliedVolts();
    }

    /**
     * 
     * @return The output <strong> current </strong> of the motor controller
     */
    public double getCurrent() {
        return io.getCurrent();
    }

    /** Unfortunately we only support <strong> celsius </strong> at this time. Do you desire farenheit or kelvins? Do the math yourself :) <p>
     * Standard conversions: <ul>
     * <li> Celsius to Farenheit: (celsius * 1.8) + 32
     * <li> Celsius to Kelvin: celsius + 273.15
     * <li> Farenheit to Kelvin: (farenheit + 459.67) / 1.8
     * <li> Kelvin to Farenheit: (kelvin * 1.8) - 459.67
     * </ul>
     * 
     * @return The temperature of the motor in <strong> celsius
     */
    public double getMotorTemp(){
        return io.getMotorTemp();
    }

    /**
     * 
     * @return The velocity of the motor in <strong> radians per second
     */
    public double getVelocityRadPerSec(){
        return io.getVelocityRadPerSec();
    }

    /**
     * 
     * @return The velocity of the motor in <strong> rotations per minute </strong> (RPM)
     */
    public double getVelocityRPM(){
        return io.getVelocityRPM();
    }

    /**
     * 
     * @return The position of the motor in <strong> degrees
     */
    public double getPositionDegrees(){
        return io.getPostiionDegrees();
    }

    /**
     * 
     * @return The position of the motor in <strong> radians
     */
    public double getPositionRadians(){
        return io.getPositionRadians();
    }

    /**
     * 
     * @return The position of the motor in <strong> rotations
     */
    public double getPositionRotations(){
        return io.getPositionRotations();
    }
    
    /** Clamped from -360 to 360
     * 
     * @param degrees is the <i> rotation </i> setpoint you desire for the motor, in <strong> degrees
     */
    public void setPositionDegrees(double degrees){
        io.setPositionDegrees(degrees);
    }

    /** Clamped from -2PI to 2PI
     * 
     * @param radians is the <i> rotation </i> setpoint you desire for the motor, in <strong> radians
     */
    public void setPositionRadians(double radians){
        io.setPositionRadians(radians);
    }

    /** Clamped from -1 to 1
     * 
     * @param rotations is the rotation setpoint you desire for the motor, in <strong> rotations
     */
    public void setPositionRotations(double rotations){
        io.setPositionRotations(rotations);
    }

    /** 
     * 
     * @return The rotation setpoint of the motor, in <strong> degrees
     */
    public double getSetpointDegrees(){
        return io.getSetpointDegrees();
    }

    /**
     * 
     * @return The rotation setpoint of the motor, in <strong> radians
     */
    public double getSetpointRadians(){
        return io.getSetpointRadians();
    }

    /** Get setpoint returns the default units for whichever control type we are using <p>
     * For velocity control <ul>
     * <li>@return The velocity setpoint of the motor, in <strong> rotations per minute </strong> (RPM)
     * </ul>
     * For rotation control <ul>
     * <li>@return The position setpoint of the motor, in <strong> rotations
     * </ul>
     */
    public double getSetpoint(){
        return io.getSetpoint();
    }

    /** Clamped depending on the motor used <ul>
     * <li>Vortex: -6784 to 6784 
     * <li>NeoV1 and NeoV2: -5676 to 5676
     * <li>Neo550: -11000 to 11000
     * </ul>
     * @param speed is the velocity setpoint of the motor, in <strong> rotations per minute </strong>(RPM)
     */
    public void setSpeedRPM(double speed){
        io.setSpeedRPM(speed);
    }
}
