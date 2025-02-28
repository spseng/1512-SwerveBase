package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private double _previousPosition;
    private int _rotationCount;
    
    public Elevator() {
        _elevatorLeaderMotor = new SparkMax(RobotMap.CAN.ELEVATOR_MOTOR_LEFT_CAN, MotorType.kBrushless);
        _elevatorFollowerMotor = new SparkMax(RobotMap.CAN.ELEVATOR_MOTOR_RIGHT_CAN, MotorType.kBrushless);

        _elevatorLeaderMotorConfig = new SparkMaxConfig();
        _elevatorFollowerMotorConfig = new SparkMaxConfig();

        _elevatorFollowerMotorConfig.follow(_elevatorLeaderMotor, true);

        _elevatorEncoder = _elevatorLeaderMotor.getAbsoluteEncoder();

        _elevatorPIDController = new PIDController(Constants.Elevator.ELEVATOR_POSITION_KP, Constants.Elevator.ELEVATOR_POSITION_KI, Constants.Elevator.ELEVATOR_POSITION_KD);
        _elevatorPIDController.setTolerance(Constants.Elevator.ELEVATOR_TOLERANCE);

        _elevatorLeaderMotorConfig.smartCurrentLimit(Constants.Elevator.ELEVATOR_CURRENT_LIMIT);
        _elevatorLeaderMotorConfig.signals.primaryEncoderPositionPeriodMs(5);

        _elevatorLeaderMotor.configure(_elevatorLeaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _elevatorFollowerMotor.configure(_elevatorFollowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _desiredHeight = 0;

        _previousPosition = _elevatorEncoder.getPosition();
        _rotationCount = 0;
    }

    public void setTargetHeight(double height) {
        double clampedHeight = Math.max(0, Math.min(Constants.Elevator.ELEVATOR_MAX_HEIGHT, height));
        _desiredHeight = clampedHeight;
    }

    @Override
    public void periodic() {
        super.periodic();
        updateEncoderRotation();
        updateMotorPower();
        SmartDashboard.putNumber("Elevator Height", getCurrentHeight());
        SmartDashboard.putNumber("Elevator Desired Height", _desiredHeight);
        SmartDashboard.updateValues();
    }

    public double getCurrentHeight() {
        return (_rotationCount + _elevatorEncoder.getPosition()) * (2.75 * Math.PI);
    }
    
    public boolean isAtTarget(){
        return Math.abs(_desiredHeight - getCurrentHeight()) < Constants.Elevator.ELEVATOR_TOLERANCE;
    }

    public void stop() {
        _elevatorLeaderMotor.stopMotor();
    }

    private void updateMotorPower() {
        double output = _elevatorPIDController.calculate(getCurrentHeight(), _desiredHeight);
        _elevatorLeaderMotor.set(-output);
    }
    
    private void updateEncoderRotation() {
        double _currentPosition = _elevatorEncoder.getPosition();
        if (_currentPosition - _previousPosition < -0.5) {
            _rotationCount++;
        } else if (_currentPosition - _previousPosition > 0.5) {
            _rotationCount--;
        }
        _previousPosition = _currentPosition;
    }
}
