package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SwerveModule extends SubsystemBase {

    // Instance Variables


    private final CANSparkMax _steerMotor;
    private final CANSparkMax _driveMotor;

    private final SparkPIDController _steeringController;
    private final SparkPIDController _driveController;

    private final AbsoluteEncoder _steerAbsoluteEncoder;

    private final RelativeEncoder _driveEncoder;

    private SwerveModuleState _desiredModuleState = new SwerveModuleState(0.0, new Rotation2d());

    private double _chassisAngularOffset;
     
    private final Translation2d _moduleLoaction;



    public SwerveModule(int steerPort, int drivePort, ModuleConfiguration config) {
        _chassisAngularOffset = config.encoderOffset;
        _steerMotor = new CANSparkMax(steerPort, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        _driveMotor = new CANSparkMax(drivePort, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        _moduleLoaction = config.position;

        _driveMotor.restoreFactoryDefaults();
        _steerMotor.restoreFactoryDefaults();

        _driveEncoder = _driveMotor.getEncoder();
        _steerAbsoluteEncoder = _steerMotor.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);

        _driveController = _driveMotor.getPIDController();
        _steeringController = _steerMotor.getPIDController();

        _driveController.setFeedbackDevice(_driveEncoder);
        _steeringController.setFeedbackDevice(_steerAbsoluteEncoder);

        _steerAbsoluteEncoder.setPositionConversionFactor(Constants.Drivetrain.STEER_POSITION_FACTOR); //TODO finish this
        _steerAbsoluteEncoder.setVelocityConversionFactor(Constants.Drivetrain.STEER_VELOCITY_FACTOR);

        _driveEncoder.setPositionConversionFactor(Constants.Drivetrain.DRIVE_POSITION_FACTOR);
        _driveEncoder.setVelocityConversionFactor(Constants.Drivetrain.DRIVE_VELOCITY_FACTOR);

        _steerAbsoluteEncoder.setInverted(Constants.Drivetrain.IS_INVERTED);

        _steeringController.setPositionPIDWrappingEnabled(true);
        _steeringController.setPositionPIDWrappingMinInput(Constants.Drivetrain.POSITION_WRAPPING_MIN_INPUT);
        _steeringController.setPositionPIDWrappingMaxInput(Constants.Drivetrain.POSITION_WRAPPING_MAX_INPUT);

        _steeringController.setP(Constants.Drivetrain.STEER_KP);
        _steeringController.setI(Constants.Drivetrain.STEER_KI);
        _steeringController.setD(Constants.Drivetrain.STEER_KD);
        _steeringController.setFF(Constants.Drivetrain.STEER_FF);
        _steeringController.setOutputRange(Constants.Drivetrain.MOTOR_MIN_OUTPUT,Constants.Drivetrain.MOTOR_MAX_OUTPUT); 
        _driveController.setP(Constants.Drivetrain.DRIVE_KP);
        _driveController.setI(Constants.Drivetrain.DRIVE_KI);
        _driveController.setD(Constants.Drivetrain.DRIVE_KD);
        _driveController.setFF(Constants.Drivetrain.DRIVE_FF);
        _driveController.setOutputRange(Constants.Drivetrain.MOTOR_MIN_OUTPUT, Constants.Drivetrain.MOTOR_MAX_OUTPUT);

       _driveMotor.setIdleMode(Constants.Drivetrain.DRIVE_IDLE_MODE);
       _steerMotor.setIdleMode(Constants.Drivetrain.STEER_IDLE_MODE);
       _driveMotor.setSmartCurrentLimit(Constants.Drivetrain.DRIVE_CURRENT_LIMIT);
       _steerMotor.setSmartCurrentLimit(Constants.Drivetrain.STEER_CURRENT_LIMIT);

       _driveMotor.burnFlash();
       _steerMotor.burnFlash();


        _desiredModuleState.angle = new Rotation2d(_steerAbsoluteEncoder.getPosition());
        _driveEncoder.setPosition(0);
     }


    public void zeroPosition() {
        _driveEncoder.setPosition(0);
    }
    public void setModuleState(SwerveModuleState state){
        SwerveModuleState correctedState = new SwerveModuleState();
        correctedState.speedMetersPerSecond = state.speedMetersPerSecond;
        correctedState.angle = state.angle.plus(Rotation2d.fromRadians(_chassisAngularOffset));

        SwerveModuleState optimizedState = SwerveModuleState.optimize(correctedState, new Rotation2d(_steerAbsoluteEncoder.getPosition()));

        _driveController.setReference(optimizedState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        _steeringController.setReference(optimizedState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        _desiredModuleState = state;     
    }
     public SwerveModulePosition getSwervePosition(){
         return new SwerveModulePosition(
            _driveEncoder.getPosition(), new Rotation2d((_steerAbsoluteEncoder.getPosition() - _chassisAngularOffset)));
     }
     public double getChassisAngularOffset(){
        return _chassisAngularOffset;
     }

    public SwerveModuleState getSwerveModuleState(){
        return new SwerveModuleState(_driveEncoder.getVelocity(), new Rotation2d(_steerAbsoluteEncoder.getPosition()- _chassisAngularOffset));
    }
    public Translation2d getSwerveModuleLocation(){
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