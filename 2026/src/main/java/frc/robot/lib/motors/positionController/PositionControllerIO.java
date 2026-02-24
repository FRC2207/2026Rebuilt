package frc.robot.lib.motors.positionController;

import org.littletonrobotics.junction.AutoLog;

public interface PositionControllerIO {
    @AutoLog
    public static class PositionControllerIOInputs {
        public double motorAppliedVolts = 0.0;
        public double[] motorCurrentAmps = new double[] {};
        public double motorEncoder = 0.0;
        public double testingNumber = 0.0;
        public double motorTemp = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(PositionControllerIOInputs inputs) {}

    /** Run the motor at the specified voltage. */
    public default void setMotorVoltage(double volts) {}

    public abstract double getEncoder();

    public abstract double getVelocity();

    /** Sets the motor position in rotations */
    public abstract void setMotorPosition(double rotations);

    /** Sets the motor position in degrees */
    public abstract void setMotorPositionDegrees(double degrees);

    /** Sets the motor position in radians */
    public abstract void setMotorPositionRadians(double radians);

    /** Returns the motor setpoint in rotations */
    public abstract double getMotorSetpoint();

    /** Returns the motor setpoint in degrees */
    public abstract double getMotorSetpointDegrees();

    /** Returns the motor setpoint in radians */
    public abstract double getMotorSetpointRadians();
}
