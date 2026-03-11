package frc.robot.lib.motors.motorController;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

public class MotorIOSparkMax implements MotorControllerIO{
    private final SparkMax motor;

    private final RelativeEncoder motorEncoder;

    private Boolean thresholdMet = false;
    private double currentThreshold;

    public MotorIOSparkMax(int deviceId, SparkMaxConfig motorConfig, double currentThreshold) {
        this.currentThreshold = currentThreshold;

        motor = new SparkMax(deviceId, MotorType.kBrushless);

        motorEncoder = motor.getEncoder();
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(MotorControllerIOInputs inputs) {
        inputs.motorAppliedVolts = motor.getAppliedOutput();
        inputs.motorCurrentAmps = motor.getOutputCurrent();
        inputs.motorEncoderRotations = getEncoderRotations();
        inputs.motorTemp = motor.getMotorTemperature();

        inputs.motorVelocityRadsPerSec = getVelocityRadPerSec();
        inputs.motorVelocityRotationPerMinute = getVelocityRPM();
    }

    @Override
    public void setMotorVoltage(double volts) {
        volts 
        
        = MathUtil.clamp(volts, -12, 12);
        motor.setVoltage(volts);
    }

    public void setMotorPercent(double percent) {
        percent = MathUtil.clamp(percent, -1, 1);
        motor.set(percent);
    }

    public double getEncoderRotations() {
        return motorEncoder.getPosition();
    }

    public boolean assessCurrent() {
        if (motor.getOutputCurrent() > currentThreshold) {
            thresholdMet = true;
        } else {
            thresholdMet = false;
        }

        return thresholdMet;
    }

    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    public double getVelocityRadPerSec() {
        return motorEncoder.getVelocity() * ((2 * Math.PI) / 60);
    }

    public double getVelocityRPM() {
        return motorEncoder.getVelocity();
    }
}
