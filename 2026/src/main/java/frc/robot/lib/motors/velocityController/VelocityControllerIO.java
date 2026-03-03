package frc.robot.lib.motors.velocityController;

import org.littletonrobotics.junction.AutoLog;

public interface VelocityControllerIO {
    @AutoLog
    public static class VelocityControllerIOInputs {
        public double motorAppliedVolts = 0.0;

        public double[] motorCurrentAmps = new double[] {};

        public double motorVelocityRadsPerSec = 0.0;

        public double motorVelocityRotationPerMinute = 0.0;

        public double motorDesiredSetpoint;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(VelocityControllerIOInputs inputs) {}

    /** Run the motor at the specified voltage. */
    public default void setMotorVoltage(double volts) {}

    /** Set the motor to a specified percent */
    public default void setPercent(double percent) {}

    /** Set the motor to a specified speed in RPM's */
    public default void setSpeed(double speed) {}
}