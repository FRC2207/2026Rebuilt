package frc.robot.lib.motors.positionController;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionController extends SubsystemBase {
    private PositionControllerIO io;
    private final PositionControllerIOInputsAutoLogged inputs = new PositionControllerIOInputsAutoLogged();
    private String loggingKey;

    /**
     * 
     * @param io         the io object for the positionController
     * @param loggingKey the path where inputs will be logged
     * 
     */
    public PositionController(PositionControllerIO io, String loggingKey) {
        this.io = io;
        this.loggingKey = loggingKey;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs(loggingKey, inputs);
    }


    public double getAngle() {
        return io.getEncoder();
    }

    public double getVelocityRPM() {
        return io.getVelocityRPM();
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
