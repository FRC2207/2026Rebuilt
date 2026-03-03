package frc.robot.lib.motors.motorController;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;

public class MotorIOSparkFlex implements MotorControllerIO{
    private final SparkFlex motor;

    private final SparkAbsoluteEncoder motorEncoder;

    private Boolean thresholdMet = false;
    private double currentThreshold;

    public MotorIOSparkFlex(int deviceId, SparkFlexConfig motorConfig, double currentThreshold) {
        this.currentThreshold = currentThreshold;

        motor = new SparkFlex(deviceId, MotorType.kBrushless);

        motorEncoder = motor.getAbsoluteEncoder();
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(MotorControllerIOInputs inputs) {
        inputs.motorAppliedVolts = motor.getAppliedOutput();
        inputs.motorCurrentAmps = new double[] {motor.getOutputCurrent()};
        inputs.motorEncoder = getEncoder();
        inputs.motorTemp = motor.getMotorTemperature();
    }

    @Override
    public void setMotorVoltage(double volts) {
        volts 
        
        = MathUtil.clamp(volts, -12, 12);
        motor.setVoltage(volts);
    }

    public void setMotor(double percent) {
        percent = MathUtil.clamp(percent, -1, 1);
        motor.set(percent);
    }

    public double getEncoder() {
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
}
