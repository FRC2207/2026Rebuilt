package frc.robot.lib.motors.velocityController;

import org.littletonrobotics.junction.AutoLog;

public interface VelocityControllerIO {
    @AutoLog
    public static class VelocityControllerIOInputs {
        public double motorAppliedVolts = 0.0;
        public double motorCurrentAmps = 0.0;
        public double motorTemp = 0.0;
        
        public double motorVelocityRadsPerSec = 0.0;
        public double motorVelocityRotationPerMinute = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(VelocityControllerIOInputs inputs) {}

    public abstract void setMotorVoltage(double volts);
    public abstract void setMotorPercent(double percent);

    public abstract double getCurrent();

    public abstract double getVelocityRPM();
    public abstract double getVelocityRadPerSec();

    /** Set the motor to a specified speed in RPM's */
    public abstract void setSpeedRPM(double speed);
}