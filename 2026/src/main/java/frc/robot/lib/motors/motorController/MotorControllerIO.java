package frc.robot.lib.motors.motorController;

import org.littletonrobotics.junction.AutoLog;

public interface MotorControllerIO {
    @AutoLog
    public static class MotorControllerIOInputs {
        public double motorAppliedVolts = 0.0;
        public double motorCurrentAmps = 0.0;
        public double motorEncoderRotations = 0.0;
        public double motorTemp = 0.0;

        public double motorVelocityRadsPerSec = 0.0;
        public double motorVelocityRotationPerMinute = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(MotorControllerIOInputs inputs) {}

    public abstract void setMotorVoltage(double volts);
    public abstract void setMotorPercent(double percent);

    public abstract double getEncoderRotations();

    public abstract boolean assessCurrent();
    public abstract double getCurrent();

    public abstract double getVelocityRadPerSec();
    public abstract double getVelocityRPM();
}
