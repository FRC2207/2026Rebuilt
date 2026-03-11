package frc.robot.lib.motors.positionController;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

public class PositionIOSparkFlex implements PositionControllerIO{
    private final SparkFlex motor;
    private final double pivotOffset;

    private final SparkAbsoluteEncoder motorEncoder;
    private SparkClosedLoopController pidController;

    public PositionIOSparkFlex(int deviceId, SparkFlexConfig motorConfig, double pivotOffset) {
        this.pivotOffset = pivotOffset;
        motor = new SparkFlex(deviceId, MotorType.kBrushless);

        motorEncoder = motor.getAbsoluteEncoder();
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController = motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(PositionControllerIOInputs inputs) {
        inputs.motorAppliedVolts = motor.getAppliedOutput();
        inputs.motorCurrentAmps = getCurrent();
        inputs.motorEncoder = getEncoder();
        inputs.motorTemp = motor.getMotorTemperature();

    }

    public double getEncoder() {
        return (motorEncoder.getPosition() * 360) + pivotOffset;
    }

    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    public double getVelocityRPM() {
        return motorEncoder.getVelocity();
    }

    /** Sets the motor position in rotations */
    public void setMotorPosition(double setpoint) {
        pidController.setSetpoint(setpoint, SparkBase.ControlType.kPosition);
    }

    /** Sets the motor position in degrees */
    public void setMotorPositionDegrees(double setpoint) {
        setMotorPosition(setpoint / 360);
    }

    /** Sets the motor position in radians */
    public void setMotorPositionRadians(double setpoint) {
        setMotorPosition(setpoint / (2 * Math.PI));
    }

    /** Returns the motor setpoint in rotations */
    public double getMotorSetpoint() {
        return pidController.getSetpoint();
    }

    /** Returns the motor setpoint in degrees */
    public double getMotorSetpointDegrees() {
        return getMotorSetpoint() * 360;
    }

    /** Returns the motor setpoint in radians */
    public double getMotorSetpointRadians() {
        return getMotorSetpoint() * 2 * Math.PI;
    }
}
