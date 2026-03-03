package frc.robot.lib.motors.velocityController;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

public class VelocityIOSparkMax implements VelocityControllerIO{
    private final SparkMax motor;
    private final SparkClosedLoopController pidController;
    private final RelativeEncoder motorEncoder;

    public VelocityIOSparkMax(int deviceId, SparkMaxConfig motorConfig) {
        motor = new SparkMax(deviceId, MotorType.kBrushless);
        pidController = motor.getClosedLoopController();

        motorEncoder = motor.getEncoder();
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
    }

    @Override
    public void updateInputs(VelocityControllerIOInputs inputs) {
        inputs.motorAppliedVolts = motor.getAppliedOutput();
        inputs.motorCurrentAmps = new double[] {motor.getOutputCurrent()};
        inputs.motorVelocityRadsPerSec = getVelocityRadians();
        inputs.motorVelocityRotationPerMinute = getVelocityRPM();
        inputs.motorDesiredSetpoint = pidController.getSetpoint();
    }

    @Override
    public void setMotorVoltage(double volts) {
        volts = MathUtil.clamp(volts, -12, 12);
        motor.setVoltage(volts);
    }

    /** Requires a decimal percentage */
    public void setPercent(double percent) {
        percent = MathUtil.clamp(percent, -1, 1);
        motor.set(percent);
    }

    /** Set speed to a specified RPM */
    public void setSpeed(double speed) {
        pidController.setSetpoint(speed, ControlType.kVelocity);
    }

    private double getVelocityRadians() {
        // motorEncoder.getVelocity() returns RPM, convert to radians per second
        return motorEncoder.getVelocity() * ((2 * Math.PI) / 60);
    }

    private double getVelocityRPM() {

        return motorEncoder.getVelocity();
    }

}
