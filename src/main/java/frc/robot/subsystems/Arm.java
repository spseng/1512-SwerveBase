package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
    private final SparkMax _armMotor;
    private final SparkAbsoluteEncoder _armEncoder;
    private final PIDController _armPIDController;
    private final SparkMaxConfig _armMotorConfig;

    private final double _armOffset = Constants.Arm.ARM_OFFSET;
    private double _setpoint;

    private static final double K_FF = 0.1;

    public Arm() {
        _armMotor = new SparkMax(RobotMap.CAN.ARM_MOTOR_CAN, MotorType.kBrushless);
        _armEncoder = _armMotor.getAbsoluteEncoder();
        _armPIDController = new PIDController(Constants.Arm.ARM_POSITION_KP, 
                                              Constants.Arm.ARM_POSITION_KI, 
                                              Constants.Arm.ARM_POSITION_KD);

        _armMotorConfig = new SparkMaxConfig();
        _armMotorConfig.absoluteEncoder.inverted(Constants.Arm.ARM_ENCODER_INVERTED);

        _armMotorConfig.idleMode(IdleMode.kBrake);

        _armMotor.configure(_armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _setpoint = getCurrentAngle();
    }

    public double getCurrentAngle() {
        return _armEncoder.getPosition(); // TODO: scaling??
    }

    public void setArmPosition(double position) {
        _setpoint = Math.max(Constants.Arm.ARM_MIN_ANGLE, Math.min(Constants.Arm.ARM_MAX_ANGLE, position));
    }

    public double getSetpoint() {
        return _setpoint;
    }

    public boolean isAtTarget() {
        return Math.abs(getCurrentAngle() - _setpoint) < Constants.Arm.ARM_TOLERANCE;
    }

    @Override
    public void periodic() {
        //super.periodic();
        updateMotorPower();
        updateSmartDashboard();
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Arm Setpoint", _setpoint);
        SmartDashboard.putNumber("Arm Position", getCurrentAngle());
        //SmartDashboard.putNumber("Arm PID Output", _armPIDController.calculate(getCurrentAngle(), _setpoint));
        //SmartDashboard.putNumber("Arm FF Output", calculateFeedforward());
    }

    private void updateMotorPower() {
        double pidOutput = -_armPIDController.calculate(getCurrentAngle(), _setpoint);
        double ffOutput = -calculateFeedforward();
        double totalOutput = pidOutput + ffOutput;
        //double totalOutput = ffOutput;
        totalOutput = Math.max(-1.0, Math.min(1.0, totalOutput));
        double sinOfAngle = Math.sin(getCurrentAngle() * 2 * Math.PI);
        SmartDashboard.putNumber("Arm Sin of Angle", sinOfAngle);
        SmartDashboard.putNumber("Arm Total Output", totalOutput);
        SmartDashboard.putNumber("Arm PID Output", pidOutput);
        SmartDashboard.putNumber("Arm FF Output", ffOutput);
        _armMotor.set(totalOutput);
    }

    private double calculateFeedforward() {
        double angle = getCurrentAngle() - _armOffset;
        return K_FF * Math.sin(angle * 2 * Math.PI);
    }
    public boolean isColision(){
        return (getCurrentAngle() < 0.159 );
    }
}