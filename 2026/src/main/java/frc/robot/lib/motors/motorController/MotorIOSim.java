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

    private double internalAppliedVolts = 0.0;
    private double clampedAppliedVolts = 0.0;
    private final double m_kS;
    private final double m_kV;
    private double ffVolts = 0.0;

    private ControlType controlType;

    private PIDController pidController;

    public enum MotorModelSim {
        Vortex, NeoV1, NeoV2, Neo550
    }

    public enum ControlType {
        Simple, Position, Velocity
    }

    /**
     * 
     * @param motorModel The model of the motor, supported models<ul> 
     * <ls> {@link MotorModelSim#Vortex}
     * <ls> {@link MotorModelSim#NeoV1}
     * <ls> {@link MotorModelSim#NeoV2}
     * <ls> {@link MotorModelSim#Neo550}
     * </ul>
     * @param controlType The type of control the motor is using <ul>
     * <ls> {@link ControlType#Simple}
     * <ls> {@link ControlType#Postion}
     * <ls> {@link ControlType#Velocity}
     * </ul>
     * @param kP The P value of PID
     * @param kI The I value of PID
     * @param kD The D value of PID
     * @param kS The S value of PIDFF
     * @param kV The V value of PIDFF
     * @param kMomentOfInertia The moment of intertia J of the DCmotor, used in {@link LinearSystemId#createDCMotorSystem(DCMotor, double, double)}
     * @param gearReduction The reduction between motor and drum, as a ratio of output to input {@link LinearSystemId#createDCMotorSystem(DCMotor, double, double)}
     * 
     */
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
            case Position:
                internalAppliedVolts = pidController.calculate(getPositionRotations());
                break;
            case Velocity:
                internalAppliedVolts = ffVolts + pidController.calculate(getVelocityRPM());
                break;
            default:
                pidController.reset();
                break;
        }
        clampedAppliedVolts = MathUtil.clamp(internalAppliedVolts,-12.0, 12.0);

        motorSim.setInputVoltage(clampedAppliedVolts);
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

    @Override
    public void setMotorPercent(double percent) {
        internalAppliedVolts = percent * 12;
    }

    @Override
    public void setMotorVoltage(double voltage) {
        internalAppliedVolts = voltage;
    }

    @Override
    public double getAppliedVolts() {
        return clampedAppliedVolts;
    }

    @Override
    public double getCurrent() {
        return Math.abs(motorSim.getCurrentDrawAmps());
    }

    @Override
    public double getMotorTemp() {
        return 0.0;
    }

    @Override
    public double getVelocityRadPerSec() {
        return motorSim.getAngularVelocityRadPerSec();
    }

    @Override
    public double getVelocityRPM() {
        return motorSim.getAngularVelocityRPM();
    }

    @Override
    public double getPostiionDegrees() {
        return motorSim.getAngularPositionRotations() * 360;
    }

    @Override
    public double getPositionRadians() {
        return motorSim.getAngularPositionRad();
    }

    @Override
    public double getPositionRotations() {
        return motorSim.getAngularPositionRotations();
    }

    @Override
    public void setPositionDegrees(double degrees) {
        pidController.setSetpoint(degrees / 360);
    }

    @Override
    public void setPositionRadians(double radians) {
        pidController.setSetpoint(radians / (2 * Math.PI));
    }

    @Override
    public void setPositionRotations(double rotations) {
        pidController.setSetpoint(rotations);
    }

    @Override
    public double getSetpointDegrees() {
        return pidController.getSetpoint() * 360;
    }

    @Override
    public double getSetpointRadians() {
        return pidController.getSetpoint() * (2 * Math.PI);
    }

    @Override
    public double getSetpoint() {
        return pidController.getSetpoint();
    }

    @Override
    public void setSpeedRPM(double speed) {
        ffVolts = m_kS * Math.signum(speed) + m_kV * speed;
        pidController.setSetpoint(speed);
    }

}
