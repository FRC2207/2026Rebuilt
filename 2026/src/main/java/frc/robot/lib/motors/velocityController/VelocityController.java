package frc.robot.lib.motors.velocityController;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VelocityController extends SubsystemBase{
    private VelocityControllerIO io;
    private final VelocityControllerIOInputsAutoLogged inputs = new VelocityControllerIOInputsAutoLogged();
    private String subsystem;
    private String number;

    public VelocityController(VelocityControllerIO io, String subsystem, String number) {
        this.io = io;
        this.subsystem = subsystem;
        this.number = number;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs(subsystem + number, inputs);
    }

    /** Sets the speed of the motor to a designated voltage from -12 to 12 */
    public void setVoltage(double volts) {
        io.setMotorVoltage(volts);
    }

    /** Sets the speed of the motor to a designated percent from -1 to 1 */
    public void setPercent(double percent) {
        io.setPercent(percent);
    }

    /** Sets the speed of the motor to a designated RPM */
    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }

    /** Resets the feed forward setpoint to 0 to stop motor movement. */
    public void stop() {
        io.setSpeed(0);
    }

    public double getVelocityRadians() {
        return inputs.motorVelocityRadsPerSec;
    }

    public double getVelocityRPM() {
        return inputs.motorVelocityRotationPerMinute;
    }

    public double[] getCurrent() {
        return inputs.motorCurrentAmps;
    }
}
