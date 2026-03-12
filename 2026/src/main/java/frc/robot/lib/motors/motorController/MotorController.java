package frc.robot.lib.motors.motorController;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorController extends SubsystemBase {
    private MotorControllerIO io;
    private final MotorControllerIOInputsAutoLogged inputs = new MotorControllerIOInputsAutoLogged();
    private String loggingKey;

    /**
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

    /** Sets the motor percentage from -1 to 1 */
    public void setMotorPercent(double percent) {
        io.setMotorPercent(percent);
    }

    public double getAppliedVolts(){
        return io.getAppliedVolts();
    }

    public double getCurrent() {
        return io.getCurrent();
    }

    public double getMotorTemp(){
        return io.getMotorTemp();
    }

    public double getVelocityRadPerSec(){
        return io.getVelocityRadPerSec();
    }

    public double getVelocityRPM(){
        return io.getVelocityRPM();
    }

    public double getPositionDegrees(){
        return io.getPostiionDegrees();
    }

    public double getPositionRadians(){
        return io.getPositionRadians();
    }

    public double getPositionRotations(){
        return io.getPositionRotations();
    }
    
    public void setPositionDegrees(double degrees){
        io.setPositionDegrees(degrees);
    }

    public void setPositionRadians(double radians){
        io.setPositionRadians(radians);
    }

    public void setPositionRotations(double rotations){
        io.setPositionRotations(rotations);
    }

    public double getSetpointDegrees(){
        return io.getSetpointDegrees();
    }

    public double getSetpointRadians(){
        return io.getSetpointRadians();
    }

    public double getSetpointRotations(){
        return io.getSetpointRotations();
    }

    public void setSpeedRPM(double speed){
        io.setSpeedRPM(speed);
    }
}
