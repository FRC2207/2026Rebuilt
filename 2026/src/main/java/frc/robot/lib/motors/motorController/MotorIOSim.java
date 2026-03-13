package frc.robot.lib.motors.motorController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.current.Constants;

public class MotorIOSim implements MotorControllerIO {
    private final DCMotorSim motorSim;
    private final DCMotor dcMotor;

    private double appliedVolts = 0.0;
    private final double m_kS;
    private final double m_kV;
    private double ffVolts = 0.0;

    private ControlType controlType;

    private PIDController pidController;

    public enum MotorModelSim {
        Vortex, NeoV1, NeoV2, Neo550
    }

    public enum ControlType {
        Simple, Postion, Velocity
    }

    public MotorIOSim(MotorModelSim motorModel, ControlType controlType, double kP, double kI, double kD, double kS,
            double kV, double kMomentOfInertia, double gearReduction) {
        this.controlType = controlType;

        switch (motorModel) {
            case Vortex:
                dcMotor = DCMotor.getNeoVortex(1);
                break;
            case NeoV1:
                dcMotor = DCMotor.getNEO(1);
                break;
            case NeoV2:
                dcMotor = DCMotor.getNEO(1);
                break;
            default:
                dcMotor = DCMotor.getNEO(1);
                break;
        }
        motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor, kMomentOfInertia, gearReduction),
                dcMotor);

        // The base setpoint unit for this implementaions is Rotations
        pidController = new PIDController(kP, kI, kD, Constants.loopPeriodSecs);
        m_kS = kS;
        m_kV = kV;

    }

    @Override
    public void updateInputs(MotorControllerIOInputs inputs) {
        switch (controlType) {
            case Simple:
                pidController.reset();
                break;
            case Postion:
                appliedVolts = pidController.calculate(getPositionRotations());
                break;
            case Velocity:
                appliedVolts = ffVolts + pidController.calculate(getVelocityRPM());
                break;
            default:
                pidController.reset();
                break;
        }

        motorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));

        motorSim.update(Constants.loopPeriodSecs);

        inputs.motorAppliedVolts = getAppliedVolts();
        inputs.motorCurrentAmps = getCurrent();
        inputs.motorTemp = 0.0;

        inputs.velocityRadsPerSec = getVelocityRadPerSec();
        inputs.velocityRotationPerMinute = getVelocityRPM();

        inputs.positionDegrees = getPostiionDegrees();
        inputs.positionRadians = getPositionRadians();
        inputs.positionRotations = getPositionRotations();

    }

    public void setMotorPercent(double percent) {
        appliedVolts = MathUtil.clamp(percent * 12, -12, 12);
    }

    public void setMotorVoltage(double voltage) {
        appliedVolts = MathUtil.clamp(voltage, -12, 12);
    }

    public double getAppliedVolts() {
        return appliedVolts;
    }

    public double getCurrent() {
        return Math.abs(motorSim.getCurrentDrawAmps());
    }

    public double getMotorTemp() {
        return 0.0;
    }

    public double getVelocityRadPerSec() {
        return motorSim.getAngularVelocityRadPerSec();
    }

    public double getVelocityRPM() {
        return motorSim.getAngularVelocityRPM();
    }

    public double getPostiionDegrees() {
        return motorSim.getAngularPositionRotations() * 360;
    }

    public double getPositionRadians() {
        return motorSim.getAngularPositionRad();

    }

    public double getPositionRotations() {
        return motorSim.getAngularPositionRotations();
    }

    public void setPositionDegrees(double degrees) {
        pidController.setSetpoint(degrees / 360);
    }

    public void setPositionRadians(double radians) {
        pidController.setSetpoint(radians / (2 * Math.PI));
    }

    public void setPositionRotations(double rotations) {
        pidController.setSetpoint(rotations);
    }

    public double getSetpointDegrees() {
        return pidController.getSetpoint() * 360;
    }

    public double getSetpointRadians() {
        return pidController.getSetpoint() * (2 * Math.PI);
    }

    public double getSetpointRotations() {
        return pidController.getSetpoint();
    }

    public void setSpeedRPM(double speed) {
        ffVolts = m_kS * Math.signum(speed) + m_kV * speed;
        pidController.setSetpoint(speed);
    }

}
