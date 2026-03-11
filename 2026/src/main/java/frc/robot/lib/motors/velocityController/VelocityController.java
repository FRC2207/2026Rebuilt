package frc.robot.lib.motors.velocityController;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VelocityController extends SubsystemBase {
    private VelocityControllerIO io;
    private final VelocityControllerIOInputsAutoLogged inputs = new VelocityControllerIOInputsAutoLogged();
    private String loggingKey;

    /**
     * 
     * @param io         the io object for the velocityController
     * @param loggingKey the path where inputs will be logged
     * 
     */
    public VelocityController(VelocityControllerIO io, String loggingKey) {
        this.io = io;
        this.loggingKey = loggingKey;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs(loggingKey, inputs);
    }

    /** Sets the speed of the motor to a designated voltage from -12 to 12 */
    public void setVoltage(double volts) {
        io.setMotorVoltage(volts);
    }

    /** Sets the speed of the motor to a designated percent from -1 to 1 */
    public void setPercent(double percent) {
        io.setMotorPercent(percent);
    }

    /** Sets the speed of the motor to a designated RPM */
    public void setSpeed(double speed) {
        io.setSpeedRPM(speed);
    }

    /** Resets the feed forward setpoint to 0 to stop motor movement. */
    public void stop() {
        io.setSpeedRPM(0);
    }

    public double getVelocityRadians() {
        return inputs.motorVelocityRadsPerSec;
    }

    public double getVelocityRPM() {
        return inputs.motorVelocityRotationPerMinute;
    }

    public double getCurrent() {
        return inputs.motorCurrentAmps;
    }
}
