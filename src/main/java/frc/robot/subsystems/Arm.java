package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase{

    private final SparkMax _armMotor;
    private final SparkAbsoluteEncoder _armEncoder;
    private final PIDController _armPIDController;
    private final SparkMaxConfig _armMotorConfig;

    private double _setpoint;
    
    public Arm(){
        _armMotor = new SparkMax(RobotMap.CAN.ARM_MOTOR_CAN, MotorType.kBrushed);

        _armEncoder = _armMotor.getAbsoluteEncoder();

        _armPIDController = new PIDController(Constants.Arm.ARM_POSITION_KP, Constants.Arm.ARM_POSITION_KI, Constants.Arm.ARM_POSITION_KD);

        _armMotorConfig = new SparkMaxConfig();

        _armMotorConfig.absoluteEncoder.inverted(Constants.Arm.ARM_ENCODER_INVERTED);

        _armMotor.configure(_armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _setpoint = getCurrentAngle();
    }

    public double getCurrentAngle(){
        return _armEncoder.getPosition(); //TODO Will Need to be Scaled
    }

    public void setArmPosition(double position){
        _setpoint = Math.max(Constants.Arm.ARM_MIN_ANGLE, Math.min(Constants.Arm.ARM_MAX_ANGLE, position));;
    }

    public double getSetpoint(){
        return _setpoint;
    }

    public boolean isAtTarget(){
        return Math.abs(getCurrentAngle() - _setpoint) < Constants.Arm.ARM_TOLERANCE;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        //_controller.setReference(_setpoint, ControlType.kPosition);
        updateMotorPower();
        updateSmartDashboard();
    }

    public void updateSmartDashboard(){
        SmartDashboard.putNumber("Arm Setpoint", _setpoint);
        SmartDashboard.putNumber("Arm Position", _armEncoder.getPosition());
    }

    private void updateMotorPower(){
        double output = _armPIDController.calculate(getCurrentAngle(), _setpoint);
        _armMotor.set(output);
    }
}
