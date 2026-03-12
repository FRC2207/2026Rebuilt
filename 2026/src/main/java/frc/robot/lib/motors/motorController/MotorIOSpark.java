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

public class MotorIOSpark implements MotorControllerIO{
    private final SparkBase motor;

    private final SparkAbsoluteEncoder motorEncoder;
    private SparkClosedLoopController closedLoopController;

    public enum SparkType {
        SparkFlex, SparkMax
    }

    public MotorIOSpark(int deviceId, SparkBaseConfig motorConfig, SparkType sparkType) {
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

    public void setMotorPercent(double percent) {
        percent = MathUtil.clamp(percent, -1, 1);
        motor.set(percent);
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
        closedLoopController.setSetpoint(degrees / 360, SparkBase.ControlType.kPosition);
    }

    public void setPositionRadians(double radians) {
        closedLoopController.setSetpoint(radians / (2 * Math.PI), SparkBase.ControlType.kPosition);
    }

    public void setPositionRotations(double rotations){
        closedLoopController.setSetpoint(rotations, SparkBase.ControlType.kPosition);
    }

    public double getSetpointDegrees(){
        return closedLoopController.getSetpoint() * 360;
    }

    public double getSetpointRadians(){
        return closedLoopController.getSetpoint() * (2 * Math.PI);
    }

    public double getSetpointRotations(){
        return closedLoopController.getSetpoint();
    }

    public void setSpeedRPM(double speed){
        closedLoopController.setSetpoint(speed, SparkBase.ControlType.kVelocity);
    }

}
