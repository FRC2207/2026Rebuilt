package frc.robot.lib.motors.velocityController;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class VelocityIOSim implements VelocityControllerIO {
    private DCMotor maxGearbox = DCMotor.getNEO(1);
    private SparkMax sparkMax = new SparkMax(1, MotorType.kBrushless);
    
    private SparkMaxSim motorSim = new SparkMaxSim(sparkMax, maxGearbox);

    private double motorAppliedVolts = 0.0;

    @Override
    public void updateInputs(VelocityControllerIOInputs inputs) {    
        inputs.motorAppliedVolts = motorAppliedVolts;
        inputs.motorCurrentAmps = motorSim.getMotorCurrent();
        inputs.motorVelocityRadsPerSec = getVelocityRadPerSec();
        inputs.motorVelocityRotationPerMinute = getVelocityRPM();
    }

    public void simulationPeriodic() {
        motorSim.iterate(getVelocityRadPerSec(), getVBus(), 0.02);
    }

    public void periodic() {
        motorSim.iterate(getVelocityRadPerSec(), getVBus(), 0.02);
    }

    public double getVelocityRadPerSec() {
        // motorSim.getVelocity returns RPM, convert to radians per second
        return motorSim.getVelocity() * ((2 * Math.PI) / 60);
    }

    public double getVelocityRPM(){
        return motorSim.getVelocity();
    }

    private double getVBus() {
        return motorSim.getBusVoltage();
    }

    public void setMotorVoltage(double volts) {
        motorAppliedVolts = MathUtil.clamp(volts, -12, 12);
        motorSim.setBusVoltage(motorAppliedVolts);
    }

    public void setMotorPercent(double percent){
        motorSim.setAppliedOutput(percent);
    }

    public void setSpeedRPM(double speed){
        motorSim.setVelocity(speed);
    }

    public double getCurrent(){
        return motorSim.getMotorCurrent();
    }
}
