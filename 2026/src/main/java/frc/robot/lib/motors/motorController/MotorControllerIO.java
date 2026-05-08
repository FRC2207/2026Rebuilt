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

    /** Sets the motor percentage from -1 to 1 */
    public default void setMotorPercent(double percent) {};

    public default void setMotorVoltage(double voltage) {};

    public default double getAppliedVolts() {return 0.0;};

    public default double getCurrent() {return 0.0;};

    public default double getMotorTemp() {return 0.0;};

    public default double getVelocityRadPerSec() {return 0.0;};
    public default double getVelocityRPM() {return 0.0;};

    public default double getPostiionDegrees() {return 0.0;};
    public default double getPositionRadians() {return 0.0;};
    public default double getPositionRotations() {return 0.0;};

    public default void setPositionDegrees(double degrees) {};
    public default void setPositionRadians(double radians) {};
    public default void setPositionRotations(double rotations) {};


    public default double getSetpointDegrees() {return 0.0;};
    public default double getSetpointRadians() {return 0.0;};
    public default double getSetpoint() {return 0.0;};

    public default void setSpeedRPM(double speed) {};

    public default void resetEncoder() {};
}
