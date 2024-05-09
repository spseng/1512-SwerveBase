package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import static frc.robot.Constants.Arm.*;


public class Arm extends SubsystemBase {
    private CANSparkMax _motor;
    private final CANcoder _encoder;
    
    private final DigitalInput _lowButton;
    private final DigitalInput _highButton;

    private final PIDController _angleController;
    private double goalAngle;


    public Arm(){
        _encoder = new CANcoder(RobotMap.CAN.ANGLE_ALIGNMENT_ENCODER_CAN);
        _highButton = new DigitalInput(RobotMap.DIO.IS_HIGHEST_ANGLE_BUTTON_ID);
        _lowButton= new DigitalInput(RobotMap.DIO.IS_LOWEST_ANGLE_BUTTON_ID);

        _motor = new CANSparkMax(RobotMap.CAN.ANGLE_ALIGNMENT_MOTOR_CAN, MotorType.kBrushed);

        _angleController = new PIDController(SHOOTING_ANGLE_KP, SHOOTING_ANGLE_KI, SHOOTING_ANGLE_KD);
     

        goalAngle = ARM_INTAKE_ANGLE;  
    }
    public void updateDashBoard(){
        SmartDashboard.putBoolean("Is Lowest", isLowestAngle());
        SmartDashboard.putBoolean("Is Highest", isHighestAngle());

        SmartDashboard.putNumber("Angle", getAngle());
  
    }
    public boolean isLowestAngle(){
        return !_lowButton.get();
    }
    public boolean isHighestAngle(){
        return !_highButton.get();
    }
    public double getAngle(){
        return _encoder.getAbsolutePosition().getValueAsDouble();
    }
    public void calibrate(){
        if(isLowestAngle()){
            _motor.stopMotor();
          
        }
    }
    public void StopArm(){
        _motor.set(0);
    }
    public void setMotorOut(double speed){
        _motor.set(speed);
    }
    public boolean isInverted(){
        return _motor.getInverted();
    }
    public double getGoalAngle(){
        return goalAngle;
    }
    public void setGoalAngle(double angle){
        goalAngle = angle;
    }
    public void setArmPosition(double position){
        goalAngle = position;
        if((goalAngle > getAngle() && !isHighestAngle()) || (goalAngle > getAngle() && !isLowestAngle())) {
            setMotorOut(getPIDasMotorOut(_angleController.calculate(goalAngle)));  
        }
    }
    public double getPIDasMotorOut(double PIDin){
        return (PIDin/100);
    }


    
}
