package frc.robot.lib.motors.positionController;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;

public class PositionIOSim implements PositionControllerIO{
    private DCMotor maxGearbox = DCMotor.getNEO(1);
    private SparkMax sparkMax = new SparkMax(1, MotorType.kBrushless);
    
    private SparkMaxSim motorSim = new SparkMaxSim(sparkMax, maxGearbox);

    private double motorAppliedVolts = 0.0;

    @Override
    public void updateInputs(PositionControllerIOInputs inputs) {    
        inputs.motorAppliedVolts = motorAppliedVolts;
        inputs.motorCurrentAmps = motorSim.getMotorCurrent();
        inputs.motorEncoder = getEncoder();
    }

    public void periodic() {
        motorSim.iterate(getVelocity(), getVBus(), 0.02);
    }

    public double getCurrent() {
        return motorSim.getMotorCurrent();
    }

    public double getVelocityRPM() {
        return motorSim.getVelocity();
    }

    public double getEncoder() {
        return motorSim.getAbsoluteEncoderSim().getPosition();
    }

    public double getVBus() {
        return motorSim.getBusVoltage();
    }

    public double getVelocity() {
        return motorSim.getVelocity();
    }

    public void setMotorPosition(double setpoint) {
        // pidController.setSetpoint(setpoint, SparkBase.ControlType.kPosition);
    }

    public void setMotorPositionDegrees(double degrees) {
        // setMotorPosition(degrees / 360);
    }

    public void setMotorPositionRadians(double radians) {
        // setMotorPosition(radians / (2 * Math.PI));
    }

    public double getMotorSetpoint() {
        // TODO: Implement
        return 0;
    }

    public double getMotorSetpointDegrees() {
        return getMotorSetpoint() * 360;
    }  

    public double getMotorSetpointRadians() {
        return getMotorSetpoint() * 2 * Math.PI;
    }
}