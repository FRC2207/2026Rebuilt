package frc.robot.lib.motors.motorController;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
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

    private SparkClosedLoopController closedLoopController;

    private final Supplier<Double> positionSupplier;
    private final Supplier<Double> velocitySupplier;

    public enum SparkType {
        SparkFlex, SparkMax
    }

    public enum MotorModel {
        Vortex, NeoV1, NeoV2, Neo550
    }

    /**
     * External encoders often require a board, which is either in Absolute or Alternate mode. Built-in Encoder is always relative
     */
    public enum EncoderType {
        EXTERNAL_ABSOLUTE,
        BUILTIN_RELATIVE,
    }

    /**
     * 
     * @param deviceId The CAN id of the MotorControler
     * @param motorConfig The {@linkplain SparkBaseConfig} for the motor
     * @param sparkType The type of spark motor controler <ul>
     * <ls> {@link SparkType#SparkMax}
     * <ls> {@link SparkType#SparkFlex}
     * </ul>
     * @param motorModel The model of the motor, supported models<ul> 
     * <ls> {@link MotorModel#Vortex}
     * <ls> {@link MotorModel#NeoV1}
     * <ls> {@link MotorModel#NeoV2}
     * <ls> {@link MotorModel#Neo550}
     * </ul>
     * @param encoderType The type of encoder <ul>
     * <ls> {@link EncoderType#EXTERNAL_ABSOLUTE}
     * <ls> {@link EncoderType#BUILTIN_RELATIVE}
     */
    public MotorIOSpark(int deviceId, SparkBaseConfig motorConfig, SparkType sparkType, MotorModel motorModel, EncoderType encoderType) {
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

        switch (encoderType) {
            case EXTERNAL_ABSOLUTE: 
                SparkAbsoluteEncoder absoluteEncoder = motor.getAbsoluteEncoder();
                this.positionSupplier = absoluteEncoder::getPosition;
                this.velocitySupplier = absoluteEncoder::getVelocity;
                break;
            case BUILTIN_RELATIVE: 
            default:
                RelativeEncoder relativeEncoder = motor.getEncoder();
                this.positionSupplier = relativeEncoder::getPosition;
                this.velocitySupplier = relativeEncoder::getVelocity;
                break;
        }

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

    /** Returns the position supplier from whichever encoder we are using */
    private double getPosition() {
        return positionSupplier.get();
    }

    /** Returns the velocity supplier from whichever encoder we are using */
    private double getVelocity() {
        return velocitySupplier.get();
    }

    @Override
    /** Sets the motor percentage from -1 to 1 */
    public void setMotorPercent(double percent) {
        percent = MathUtil.clamp(percent, -1, 1);
        motor.set(percent);
    }

    @Override
    public void setMotorVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        motor.setVoltage(voltage);
    }

    @Override
    public double getAppliedVolts() {
        return motor.getAppliedOutput();
    }

    @Override
    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    @Override
    public double getMotorTemp() {
        return motor.getMotorTemperature();
    }

    @Override
    public double getVelocityRadPerSec() {
        return getVelocity() * ((2 * Math.PI) / 60);
    }

    @Override
    public double getVelocityRPM() {
        return getVelocity();
    }

    @Override
    public double getPostiionDegrees() {
        return getPosition() * 360;
    }

    @Override
    public double getPositionRadians() {
        return getPosition() * (2 * Math.PI);
    }

    @Override
    public double getPositionRotations() {
        return getPosition();
    }

    @Override
    public void setPositionDegrees(double degrees) {
        double clampDegrees = MathUtil.clamp(degrees, -360, 360);
        closedLoopController.setSetpoint(clampDegrees / 360, SparkBase.ControlType.kPosition);
    }

    @Override
    public void setPositionRadians(double radians) {
        double clampRadians = MathUtil.clamp(radians, -2 * Math.PI, 2 * Math.PI);
        closedLoopController.setSetpoint(clampRadians / (2 * Math.PI), SparkBase.ControlType.kPosition);
    }

    @Override
    public void setPositionRotations(double rotations) {
        double clampRotations = MathUtil.clamp(rotations, -1, 1);
        closedLoopController.setSetpoint(clampRotations, SparkBase.ControlType.kPosition);
    }

    @Override
    public double getSetpointDegrees() {
        return closedLoopController.getSetpoint() * 360;
    }

    @Override
    public double getSetpointRadians() {
        return closedLoopController.getSetpoint() * (2 * Math.PI);
    }

    @Override
    public double getSetpoint() {
        return closedLoopController.getSetpoint();
    }

    @Override
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