package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
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
    private final SparkMax _driveMotor;

    private final SparkMaxConfig _steerMotorConfig;
    private final SparkMaxConfig _driveMotorConfig;

    private final SparkClosedLoopController _steeringController;
    private final SparkClosedLoopController _driveController;

    //private final AbsoluteEncoder _steerAbsoluteEncoder;

    //private final RelativeEncoder _driveEncoder;
    private final Translation2d _moduleLoaction;
    private SwerveModuleState _desiredModuleState = new SwerveModuleState(0.0, new Rotation2d());
    private double _chassisAngularOffset;

    public SwerveModule(int steerPort, int drivePort, ModuleConfiguration config) {
        _chassisAngularOffset = config.encoderOffset;
        _steerMotor = new SparkMax(steerPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        _driveMotor = new SparkMax(drivePort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        _moduleLoaction = config.position;

        _steerMotorConfig = new SparkMaxConfig();
        _driveMotorConfig = new SparkMaxConfig();

        //_driveMotor.restoreFactoryDefaults();
        //_steerMotor.restoreFactoryDefaults();

        //_driveEncoder = _driveMotor.getEncoder();
        //_steerAbsoluteEncoder = _steerMotor.getAbsoluteEncoder(com.revrobotics.spark.SparkAbsoluteEncoder.Type.kDutyCycle);
        //_steerAbsoluteEncoder = _driveMotor.getAbsoluteEncoder();

        _driveController = _driveMotor.getClosedLoopController();
        _steeringController = _steerMotor.getClosedLoopController();

        //_driveController.setFeedbackDevice(_driveEncoder);
        //_steeringController.setFeedbackDevice(_steerAbsoluteEncoder);

        //_steerAbsoluteEncoder.setPositionConversionFactor(Constants.Drivetrain.STEER_POSITION_FACTOR); //TODO finish this
        //_steerAbsoluteEncoder.setVelocityConversionFactor(Constants.Drivetrain.STEER_VELOCITY_FACTOR); // TODO this might be the root of problems

        //_driveEncoder.setPositionConversionFactor(Constants.Drivetrain.DRIVE_POSITION_FACTOR);
        //_driveEncoder.setVelocityConversionFactor(Constants.Drivetrain.DRIVE_VELOCITY_FACTOR);

        //_steerAbsoluteEncoder.setInverted(Constants.Drivetrain.IS_INVERTED);

        //_steeringController.setPositionPIDWrappingEnabled(true);
        //_steeringController.setPositionPIDWrappingMinInput(Constants.Drivetrain.POSITION_WRAPPING_MIN_INPUT);
        //_steeringController.setPositionPIDWrappingMaxInput(Constants.Drivetrain.POSITION_WRAPPING_MAX_INPUT);

        //_steeringController.setP(Constants.Drivetrain.STEER_KP);
        //_steeringController.setI(Constants.Drivetrain.STEER_KI);
        //_steeringController.setD(Constants.Drivetrain.STEER_KD);
        //_steeringController.setFF(Constants.Drivetrain.STEER_FF);
        //_steeringController.setOutputRange(Constants.Drivetrain.MOTOR_MIN_OUTPUT, Constants.Drivetrain.MOTOR_MAX_OUTPUT);
        //_driveController.setP(Constants.Drivetrain.DRIVE_KP);
        //_driveController.setI(Constants.Drivetrain.DRIVE_KI);
        //_driveController.setD(Constants.Drivetrain.DRIVE_KD);
        //_driveController.setFF(Constants.Drivetrain.DRIVE_FF);
        //_driveController.setOutputRange(Constants.Drivetrain.MOTOR_MIN_OUTPUT, Constants.Drivetrain.MOTOR_MAX_OUTPUT);

        //_driveMotor.setIdleMode(Constants.Drivetrain.DRIVE_IDLE_MODE);
        //_steerMotor.setIdleMode(Constants.Drivetrain.STEER_IDLE_MODE);
        //_driveMotor.setSmartCurrentLimit(Constants.Drivetrain.DRIVE_CURRENT_LIMIT);
        //_steerMotor.setSmartCurrentLimit(Constants.Drivetrain.STEER_CURRENT_LIMIT);

        //_driveMotor.burnFlash();
        //_steerMotor.burnFlash();

        _steerMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        _driveMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        

        _steerMotorConfig.encoder.positionConversionFactor(Constants.Drivetrain.STEER_POSITION_FACTOR);
        _steerMotorConfig.encoder.velocityConversionFactor(Constants.Drivetrain.STEER_VELOCITY_FACTOR);

        _steerMotorConfig.encoder.inverted(Constants.Drivetrain.IS_INVERTED);

        _steerMotorConfig.closedLoop.positionWrappingEnabled(true);
        _steerMotorConfig.closedLoop.positionWrappingMinInput(Constants.Drivetrain.POSITION_WRAPPING_MIN_INPUT);
        _steerMotorConfig.closedLoop.positionWrappingMaxInput(Constants.Drivetrain.POSITION_WRAPPING_MAX_INPUT);

        _steerMotorConfig.closedLoop.pidf(Constants.Drivetrain.STEER_KP, Constants.Drivetrain.STEER_KI, Constants.Drivetrain.STEER_KD, Constants.Drivetrain.STEER_FF);
        _driveMotorConfig.closedLoop.pidf(Constants.Drivetrain.DRIVE_KP, Constants.Drivetrain.DRIVE_KI, Constants.Drivetrain.DRIVE_KD, Constants.Drivetrain.DRIVE_FF);

        _steerMotorConfig.closedLoop.outputRange(Constants.Drivetrain.MOTOR_MIN_OUTPUT, Constants.Drivetrain.MOTOR_MAX_OUTPUT);
        _driveMotorConfig.closedLoop.outputRange(Constants.Drivetrain.MOTOR_MIN_OUTPUT, Constants.Drivetrain.MOTOR_MAX_OUTPUT);

        _steerMotorConfig.idleMode(Constants.Drivetrain.STEER_IDLE_MODE);
        _driveMotorConfig.idleMode(Constants.Drivetrain.DRIVE_IDLE_MODE);

        _steerMotorConfig.smartCurrentLimit(Constants.Drivetrain.STEER_CURRENT_LIMIT);
        _driveMotorConfig.smartCurrentLimit(Constants.Drivetrain.DRIVE_CURRENT_LIMIT);

        _steerMotor.configure(_steerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _driveMotor.configure(_driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _desiredModuleState.angle = new Rotation2d(_steerMotor.getEncoder().getPosition());
        _driveMotor.getEncoder().setPosition(0);
        //_driveEncoder.setPosition(0);
    }


    public void zeroPosition() {
        _driveMotor.getEncoder().setPosition(0);
    }

    public void setModuleState(SwerveModuleState state) {
        SwerveModuleState correctedState = new SwerveModuleState();
        correctedState.speedMetersPerSecond = state.speedMetersPerSecond;
        correctedState.angle = state.angle.plus(Rotation2d.fromRadians(_chassisAngularOffset));

        SwerveModuleState optimizedState = SwerveModuleState.optimize(correctedState, new Rotation2d(_steerMotor.getAbsoluteEncoder().getPosition()));

        _driveController.setReference(optimizedState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
        _steeringController.setReference(optimizedState.angle.getRadians(), SparkMax.ControlType.kPosition);

        _desiredModuleState = state;
    }

    public SwerveModulePosition getSwervePosition() {
        //return new SwerveModulePosition(_driveEncoder.getPosition(), new Rotation2d((_steerAbsoluteEncoder.getPosition() - _chassisAngularOffset)));
        return new SwerveModulePosition(_driveMotor.getEncoder().getPosition(), new Rotation2d((_steerMotor.getAbsoluteEncoder().getPosition() - _chassisAngularOffset)));
    }

    public double getDriveEncoder() {
        return _driveMotor.getEncoder().getPosition();
        //return _driveEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return _driveMotor.getEncoder().getVelocity();
        //return _driveEncoder.getVelocity();
    }

    public double getSteerEncoderPosition() {
        return _steerMotor.getAbsoluteEncoder().getPosition();
        //return _steerAbsoluteEncoder.getPosition();
    }

    public double getChassisAngularOffset() {
        return _chassisAngularOffset;
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(_driveMotor.getEncoder().getVelocity(), new Rotation2d(_steerMotor.getAbsoluteEncoder().getPosition() - _chassisAngularOffset));
        //return new SwerveModuleState(_driveEncoder.getVelocity(), new Rotation2d(_steerAbsoluteEncoder.getPosition() - _chassisAngularOffset));
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