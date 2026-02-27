package frc.robot.lib.motors.positionController;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionController extends SubsystemBase{
    private PositionControllerIO io;
    private final PositionControllerIOInputsAutoLogged inputs = new PositionControllerIOInputsAutoLogged();
    private String subsystem;

    public PositionController(PositionControllerIO io, String subsystem) {
        this.io = io;
        this.subsystem = subsystem;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs(subsystem + "Pivot", inputs);
    }

    public void setVoltage(double volts) {
        io.setMotorVoltage(volts);
    }

    public double getAngle() {
        return io.getEncoder();
    }

    public double getVelocity() {
        return io.getVelocity();
    }

    /** Sets the motor position in rotations */
    public void setMotorPosition(double rotations) {
        io.setMotorPosition(rotations);
    }

    /** Sets the motor position in degrees */
    public void setMotorPositionDegrees(double degrees) {
        io.setMotorPositionDegrees(degrees);
    }

    /** Sets the motor position in radians */
    public void setMotorPositionRadians(double radians) {
        io.setMotorPositionRadians(radians);
    }

    /** Returns the motor setpoint in rotations */
    public double getMotorSetpoint() {
        return io.getMotorSetpoint();
    }

    /** Returns the motor setpoint in degrees */
    public double getMotorSetpointDegrees() {
        return io.getMotorSetpointDegrees();
    }

    /** Returns the motor setpoint in radians */
    public double getMotorSetpointRadians() {
        return io.getMotorSetpointRadians();
    }
}
