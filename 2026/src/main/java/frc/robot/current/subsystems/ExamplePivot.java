package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants.IntakeConstants;
import frc.robot.lib.motors.positionController.PositionController;
import frc.robot.lib.motors.positionController.PositionIOSim;
import frc.robot.lib.motors.positionController.PositionIOSparkMax;

public class ExamplePivot extends SubsystemBase{
    private PositionController motor;

    private final int motorID = 67;

    private ProfiledPIDController motorPIDController;
    private TrapezoidProfile.Constraints motorConstraints;
    private final ArmFeedforward motorFeedForward = new ArmFeedforward(1, 0, 0, 0);

    private Boolean inDownPosition = false;

    private double requestedPivotGoal;
    // TODO: TEST THESE VALUES
    private double minAngle = 45;
    private double maxAngle = 80;

    public ExamplePivot(String robotType) {

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(40);

        motorConfig.idleMode(IdleMode.kBrake);

        motorConfig.closedLoop
            .p(IntakeConstants.kP)
            .i(IntakeConstants.kI)
            .d(IntakeConstants.kD)
        .feedForward // Set Feedforward gains for the velocity controller
            .kS(IntakeConstants.kS) // Static gain (volts)
            .kV(IntakeConstants.kV) // Velocity gain (volts per RPM)
            .kA(IntakeConstants.kA); // Acceleration gain (volts per RPM/s)

        motorConstraints = new TrapezoidProfile.Constraints(60, 30);
        motorPIDController = new ProfiledPIDController(1, 0, 0, motorConstraints);

        switch (robotType) {
            case "Real":
                motor = new PositionController(new PositionIOSparkMax(motorID, motorConfig,0), "Pivot");

                break;
            case "Sim":
                motor = new PositionController(new PositionIOSim(), "Algae");

                break;
            default:
                motor = new PositionController(new PositionIOSparkMax(motorID, motorConfig, 0), "Pivot");

                break;
        }

        requestedPivotGoal = 45;
    }

    public void periodic() {
        motor.updateInputs();
        Logger.recordOutput("AlgeaPivot/GoalVelocity", motorPIDController.getGoal().velocity);
        Logger.recordOutput("AlgaePivot/Goal", motorPIDController.getGoal().position);
        Logger.recordOutput("AlgaePivot/PIDOutput", motorPIDController.calculate(motor.getAngle())
        + motorFeedForward.calculate(motor.getAngle(), motorPIDController.getGoal().velocity));

        double outputGoal = MathUtil.clamp(requestedPivotGoal, minAngle, maxAngle);
        motorPIDController.setGoal(outputGoal);
        
        motor.setVoltage(
            motorPIDController.calculate(motor.getAngle())
                + motorFeedForward.calculate(motor.getAngle(), motorPIDController.getGoal().velocity)
            );
        
        Logger.recordOutput("AlgaePivot/RequestedGoal", requestedPivotGoal);
    }

    /**
     * Method to set the pivot goal to the current position
     */
    public void initialization() {
        requestedPivotGoal = motor.getAngle();
    }

    /**
     * @return The requested goal to set the pivot at
     */
    public double getRequestedGoal() {
        return requestedPivotGoal;
    }

    public void adjustHeight(double percent) {
        double increaseFactor = percent / 5 ;
        double currentHeight = requestedPivotGoal;

        requestedPivotGoal = currentHeight + increaseFactor;
    }

    public void setVoltage(double percent) {
        motor.setVoltage(percent * 12);
    }

    public double getAlgaeAngle() {
        return motor.getAngle();
    }

    public Boolean isDown() {
        return inDownPosition;
    }
}
