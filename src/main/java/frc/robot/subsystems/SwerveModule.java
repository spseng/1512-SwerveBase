package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SwerveModule extends SubsystemBase {

    // Instance Variables

    private final SparkMax _steerMotor;
    private final SparkFlex _driveMotor;

    private final SparkMaxConfig _steerMotorConfig;
    private final SparkFlexConfig _driveMotorConfig;

    private final SparkClosedLoopController _steeringController;
    private final SparkClosedLoopController _driveController;

    private final Translation2d _moduleLoaction;
    private SwerveModuleState _desiredModuleState = new SwerveModuleState(0.0, new Rotation2d());
    private double _chassisAngularOffset;

    public SwerveModule(int steerPort, int drivePort, ModuleConfiguration config) {
        _chassisAngularOffset = config.encoderOffset;
        _steerMotor = new SparkMax(steerPort, MotorType.kBrushless);
        _driveMotor = new SparkFlex(drivePort, MotorType.kBrushless);
        _moduleLoaction = config.position;

        _steerMotorConfig = new SparkMaxConfig();
        _driveMotorConfig = new SparkFlexConfig();

        _driveController = _driveMotor.getClosedLoopController();
        _steeringController = _steerMotor.getClosedLoopController();

        _steerMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingMinInput(Constants.Drivetrain.POSITION_WRAPPING_MIN_INPUT)
            .positionWrappingMaxInput(Constants.Drivetrain.POSITION_WRAPPING_MAX_INPUT)
            .pidf(Constants.Drivetrain.STEER_KP, Constants.Drivetrain.STEER_KI, Constants.Drivetrain.STEER_KD, Constants.Drivetrain.STEER_FF)
            .outputRange(Constants.Drivetrain.MOTOR_MIN_OUTPUT, Constants.Drivetrain.MOTOR_MAX_OUTPUT);

        _steerMotorConfig.absoluteEncoder
            .positionConversionFactor(Constants.Drivetrain.STEER_POSITION_FACTOR)
            .velocityConversionFactor(Constants.Drivetrain.STEER_VELOCITY_FACTOR)
            .inverted(Constants.Drivetrain.IS_INVERTED);

        _driveMotorConfig.encoder
            .positionConversionFactor(Constants.Drivetrain.DRIVE_POSITION_FACTOR)
            .velocityConversionFactor(Constants.Drivetrain.DRIVE_VELOCITY_FACTOR);

        _driveMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(Constants.Drivetrain.DRIVE_KP, Constants.Drivetrain.DRIVE_KI, Constants.Drivetrain.DRIVE_KD, Constants.Drivetrain.DRIVE_FF)
            .outputRange(Constants.Drivetrain.MOTOR_MIN_OUTPUT, Constants.Drivetrain.MOTOR_MAX_OUTPUT);

        _steerMotorConfig.idleMode(Constants.Drivetrain.STEER_IDLE_MODE);
        _driveMotorConfig.idleMode(Constants.Drivetrain.DRIVE_IDLE_MODE);

        _steerMotorConfig.smartCurrentLimit(Constants.Drivetrain.STEER_CURRENT_LIMIT);
        _driveMotorConfig.smartCurrentLimit(Constants.Drivetrain.DRIVE_CURRENT_LIMIT);

        _steerMotor.configure(_steerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _driveMotor.configure(_driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _desiredModuleState.angle = new Rotation2d(_steerMotor.getAbsoluteEncoder().getPosition());
        _driveMotor.getEncoder().setPosition(0);
    }


    public void zeroPosition() {
        _driveMotor.getEncoder().setPosition(0);
    }

    public void setModuleState(SwerveModuleState state) {
        SwerveModuleState correctedState = new SwerveModuleState();
        correctedState.speedMetersPerSecond = state.speedMetersPerSecond;
        correctedState.angle = state.angle.plus(Rotation2d.fromRadians(_chassisAngularOffset));

        SwerveModuleState optimizedState = SwerveModuleState.optimize(correctedState, new Rotation2d(_steerMotor.getAbsoluteEncoder().getPosition()));

        _driveController.setReference(optimizedState.speedMetersPerSecond, SparkFlex.ControlType.kVelocity);
        _steeringController.setReference(optimizedState.angle.getRadians(), SparkMax.ControlType.kPosition);

        _desiredModuleState = state;
    }

    public SwerveModulePosition getSwervePosition() {
        return new SwerveModulePosition(_driveMotor.getEncoder().getPosition(), new Rotation2d((_steerMotor.getAbsoluteEncoder().getPosition() - _chassisAngularOffset)));
    }

    public double getDriveEncoder() {
        return _driveMotor.getEncoder().getPosition();
    }

    public double getDriveVelocity(){
        return _driveMotor.getEncoder().getVelocity();
    }

    public double getSteerEncoderPosition() {
        return _steerMotor.getAbsoluteEncoder().getPosition();
    }

    public double getChassisAngularOffset() {
        return _chassisAngularOffset;
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(_driveMotor.getEncoder().getVelocity(), new Rotation2d(_steerMotor.getAbsoluteEncoder().getPosition() - _chassisAngularOffset));
    }

    public Translation2d getSwerveModuleLocation() {
        return _moduleLoaction;
    }


    public static class ModuleConfiguration {
        public String moduleName = "";

        public Translation2d position = new Translation2d();

        public double encoderOffset = 0.0;
        public boolean encoderInverted = false;

        public String canBus = "CANivore";
    }
}