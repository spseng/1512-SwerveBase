package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
    private final SparkMax _elevatorLeaderMotor;
    private final SparkMax _elevatorFollowerMotor;

    private final SparkMaxConfig _elevatorLeaderMotorConfig;
    private final SparkMaxConfig _elevatorFollowerMotorConfig;

    private final AbsoluteEncoder _elevatorEncoder;

    private final PIDController _elevatorPIDController;

    private double _desiredHeight;
    

    public Elevator() {
        _elevatorLeaderMotor = new SparkMax(RobotMap.CAN.ELEVATOR_MOTOR_LEFT, MotorType.kBrushless);
        _elevatorFollowerMotor = new SparkMax(RobotMap.CAN.ELEVATOR_MOTOR_RIGHT, MotorType.kBrushless);

        _elevatorLeaderMotorConfig = new SparkMaxConfig();
        _elevatorFollowerMotorConfig = new SparkMaxConfig();

        _elevatorFollowerMotorConfig.follow(_elevatorLeaderMotor, true);

        _elevatorEncoder = _elevatorLeaderMotor.getAbsoluteEncoder();

        _elevatorPIDController = new PIDController(Constants.Elevator.ELEVATOR_POSITION_KP, Constants.Elevator.ELEVATOR_POSITION_KI, Constants.Elevator.ELEVATOR_POSITION_KD);
        _elevatorPIDController.setTolerance(Constants.Elevator.ELEVATOR_TOLERANCE);

        _elevatorLeaderMotorConfig.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.Elevator.ELEVATOR_MOTOR_KP, Constants.Elevator.ELEVATOR_MOTOR_KI, Constants.Elevator.ELEVATOR_MOTOR_KD)
            .outputRange(Constants.Elevator.ELEVATOR_MOTOR_MIN_OUTPUT, Constants.Elevator.ELEVATOR_MOTOR_MAX_OUTPUT);

        _elevatorLeaderMotorConfig.smartCurrentLimit(Constants.Elevator.ELEVATOR_CURRENT_LIMIT);

        _elevatorLeaderMotor.configure(_elevatorLeaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _elevatorFollowerMotor.configure(_elevatorFollowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTargetHeight(double height) {
        double clampedHeight = Math.max(0, Math.min(Constants.Elevator.ELEVATOR_MAX_HEIGHT, height));
        double currentHeight = getCurrentHeight();
        double output = _elevatorPIDController.calculate(currentHeight, clampedHeight);

        _elevatorLeaderMotor.set(output);
    }

    public double getCurrentHeight() {
        return _elevatorEncoder.getPosition() * 10; //TODO implement the equation of the encoder
    }
    public boolean isAtTarget(){
        return ((_desiredHeight - Constants.Elevator.ELEVATOR_TOLERANCE < getCurrentHeight() && _desiredHeight + Constants.Elevator.ELEVATOR_TOLERANCE  > getCurrentHeight()));
    }

    public void stop() {
        _elevatorLeaderMotor.stopMotor();
    }
}
