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

    /** Sets the motor voltage from -12 to 12 */
    public void setVoltage(double volts) {
        io.setMotorVoltage(volts);
    }

    /** Sets the motor percentage from -1 to 1 */
    public void setPercent(double percent) {
        io.setMotorPercent(percent);
    }

    public boolean hasCurrentReached() {
        return io.assessCurrent();
    }

    public double getCurrent() {
        return io.getCurrent();
    }
}
