package frc.robot.lib.motors.motorController;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;

public class MotorIOSim implements MotorControllerIO{
    private final SparkSim sparkSim;
    private final DCMotor motorGearbox;

    private final SparkAbsoluteEncoderSim motorEncoder;
    // private SparkClosedLoopController closedLoopController;

    public enum SparkType {
        SparkFlex, SparkMax
    }

    public MotorIOSim(int deviceId, SparkBaseConfig motorConfig, SparkType sparkType) {
        switch (sparkType) {
            case SparkFlex:
                motorGearbox = DCMotor.getNeoVortex(1);
                sparkSim = new SparkFlexSim(new SparkFlex(deviceId, MotorType.kBrushless), motorGearbox);
                break;
            case SparkMax:
                motorGearbox = DCMotor.getNEO(1);
                sparkSim = new SparkMaxSim(new SparkMax(deviceId, MotorType.kBrushless), motorGearbox);
                break;
            default:
                motorGearbox = DCMotor.getNEO(1);
                sparkSim = new SparkMaxSim(new SparkMax(deviceId, MotorType.kBrushless), motorGearbox);
                break;
        }

        motorEncoder = sparkSim.getAbsoluteEncoderSim();
        
        // TODO: Implement
        // sparkSim.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // TODO: Implement
        // closedLoopController = sparkSim.getClosedLoopController();
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

    public void simulationPeriodic(){
        sparkSim.iterate(getVelocityRPM(), getVBus(), 0.02);
    }

    public void setMotorPercent(double percent) {
        // TODO: Implement
    }

    public double getAppliedVolts() {
        return sparkSim.getAppliedOutput();
    }

    public double getVBus() {
        return sparkSim.getBusVoltage();
    }

    public double getCurrent() {
        return sparkSim.getMotorCurrent();
    }

    public double getMotorTemp() {
        return 0.0;
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
        // TODO: Implement
        // closedLoopController.setSetpoint(degrees / 360, SparkBase.ControlType.kPosition);
    }

    public void setPositionRadians(double radians) {
        // TODO: Implement
        // closedLoopController.setSetpoint(radians / (2 * Math.PI), SparkBase.ControlType.kPosition);
    }

    public void setPositionRotations(double rotations){
        // TODO: Implement
        // closedLoopController.setSetpoint(rotations, SparkBase.ControlType.kPosition);
    }

    public double getSetpointDegrees(){
        // TODO: Implement
        return 0.0;
        // return closedLoopController.getSetpoint() * 360;
    }

    public double getSetpointRadians(){
        // TODO: Implement
        return 0.0;
        // return closedLoopController.getSetpoint() * (2 * Math.PI);
    }

    public double getSetpointRotations(){
        // TODO: Implement
        return 0.0;
        //return closedLoopController.getSetpoint();
    }

    public void setSpeedRPM(double speed){
        // TODO: Implement
        //closedLoopController.setSetpoint(speed, SparkBase.ControlType.kVelocity);
    }

}
