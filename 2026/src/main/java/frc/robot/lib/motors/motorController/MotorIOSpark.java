package frc.robot.lib.motors.motorController;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.MathUtil;

public class MotorIOSpark implements MotorControllerIO {
    private final SparkBase motor;
    private MotorModel motorModel;

    private final SparkAbsoluteEncoder motorEncoder;
    private SparkClosedLoopController closedLoopController;

    public enum SparkType {
        SparkFlex, SparkMax
    }

    public enum MotorModel {
        Vortex, NeoV1, NeoV2, Neo550
    }

    public MotorIOSpark(int deviceId, SparkBaseConfig motorConfig, SparkType sparkType, MotorModel motorModel) {
        this.motorModel = motorModel;

        switch (sparkType) {
            case SparkFlex:
                motor = new SparkFlex(deviceId, MotorType.kBrushless);
                break;
            case SparkMax:
                motor = new SparkMax(deviceId, MotorType.kBrushless);
                break;
            default:
                motor = new SparkMax(deviceId, MotorType.kBrushless);
                break;
        }

        motorEncoder = motor.getAbsoluteEncoder();
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        closedLoopController = motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(MotorControllerIOInputs inputs) {
        inputs.motorAppliedVolts = getAppliedVolts();
        inputs.motorCurrentAmps = getCurrent();
        inputs.motorTemp = getMotorTemp();

        inputs.velocityRadsPerSec = getVelocityRadPerSec();
        inputs.velocityRotationPerMinute = getVelocityRPM();

        inputs.positionDegrees = getPostiionDegrees();
        inputs.positionRadians = getPositionRadians();
        inputs.positionRotations = getPositionRotations();

    }

    /** Sets the motor percentage from -1 to 1 */
    public void setMotorPercent(double percent) {
        percent = MathUtil.clamp(percent, -1, 1);
        motor.set(percent);
    }

    public void setMotorVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        motor.setVoltage(voltage);
    }

    public double getAppliedVolts() {
        return motor.getAppliedOutput();
    }

    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    public double getMotorTemp() {
        return motor.getMotorTemperature();
    }

    public double getVelocityRadPerSec() {
        return motorEncoder.getVelocity() * ((2 * Math.PI) / 60);
    }

    public double getVelocityRPM() {
        return motorEncoder.getVelocity();
    }

    public double getPostiionDegrees() {
        return motorEncoder.getPosition() * 360;
    }

    public double getPositionRadians() {
        return motorEncoder.getPosition() * (2 * Math.PI);
    }

    public double getPositionRotations() {
        return motorEncoder.getPosition();
    }

    public void setPositionDegrees(double degrees) {
        double clampDegrees = MathUtil.clamp(degrees, -360, 360);
        closedLoopController.setSetpoint(clampDegrees / 360, SparkBase.ControlType.kPosition);
    }

    public void setPositionRadians(double radians) {
        double clampRadians = MathUtil.clamp(radians, -2 * Math.PI, 2 * Math.PI);
        closedLoopController.setSetpoint(clampRadians / (2 * Math.PI), SparkBase.ControlType.kPosition);
    }

    public void setPositionRotations(double rotations) {
        double clampRotations = MathUtil.clamp(rotations, -1, 1);
        closedLoopController.setSetpoint(clampRotations, SparkBase.ControlType.kPosition);
    }

    public double getSetpointDegrees() {
        return closedLoopController.getSetpoint() * 360;
    }

    public double getSetpointRadians() {
        return closedLoopController.getSetpoint() * (2 * Math.PI);
    }

    public double getSetpointRotations() {
        return closedLoopController.getSetpoint();
    }

    public void setSpeedRPM(double speed) {
        double clampSpeed;

        switch (motorModel) {
            case Vortex:
                clampSpeed = MathUtil.clamp(speed, -6784, 6784);
                break;
            case NeoV1:
            case NeoV2:
                clampSpeed = MathUtil.clamp(speed, -5676, 5676);
                break;
            case Neo550:
                clampSpeed = MathUtil.clamp(speed, -11000, 11000);
                break;
            default:
                clampSpeed = MathUtil.clamp(speed, -5676, 5676); // The lowest empirical stall torque of a rev motor as
                                                                     // of 2026
        }

        closedLoopController.setSetpoint(clampSpeed, SparkBase.ControlType.kVelocity);
    }

}
