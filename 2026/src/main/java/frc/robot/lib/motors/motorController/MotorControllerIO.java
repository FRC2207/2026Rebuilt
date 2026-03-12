package frc.robot.lib.motors.motorController;

import org.littletonrobotics.junction.AutoLog;

public interface MotorControllerIO {
    @AutoLog
    public static class MotorControllerIOInputs {
        public double motorAppliedVolts = 0.0;
        public double motorCurrentAmps = 0.0;
        public double motorTemp = 0.0;

        public double velocityRadsPerSec = 0.0;
        public double velocityRotationPerMinute = 0.0;

        public double positionDegrees = 0.0;
        public double positionRadians = 0.0;
        public double positionRotations = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(MotorControllerIOInputs inputs) {}

    public abstract void setMotorPercent(double percent);

    public abstract double getAppliedVolts();

    public abstract double getCurrent();

    public abstract double getMotorTemp();

    public abstract double getVelocityRadPerSec();
    public abstract double getVelocityRPM();

    public abstract double getPostiionDegrees();
    public abstract double getPositionRadians();
    public abstract double getPositionRotations();

    public abstract void setPositionDegrees(double degrees);
    public abstract void setPositionRadians(double radians);
    public abstract void setPositionRotations(double rotations);


    public abstract double getSetpointDegrees();
    public abstract double getSetpointRadians();
    public abstract double getSetpointRotations();

    public abstract void setSpeedRPM(double speed);
}
