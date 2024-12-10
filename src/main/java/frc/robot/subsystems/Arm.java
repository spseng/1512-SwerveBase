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
    private double motorOut;
    

    public Arm(){
        _encoder = new CANcoder(RobotMap.CAN.ANGLE_ALIGNMENT_ENCODER_CAN);
        _highButton = new DigitalInput(RobotMap.DIO.IS_HIGHEST_ANGLE_BUTTON_ID);
        _lowButton= new DigitalInput(RobotMap.DIO.IS_LOWEST_ANGLE_BUTTON_ID);

        _motor = new CANSparkMax(RobotMap.CAN.ANGLE_ALIGNMENT_MOTOR_CAN, MotorType.kBrushed);

        _angleController = new PIDController(SHOOTING_ANGLE_KP, SHOOTING_ANGLE_KI, SHOOTING_ANGLE_KD);

        goalAngle = ARM_INTAKE_ANGLE;  
        _encoder.setPosition(0.0);
        
    }
    public void updateDashBoard(){
        SmartDashboard.putBoolean("Is Lowest", isLowestAngle());
        SmartDashboard.putBoolean("Is Highest", isHighestAngle());

        SmartDashboard.putNumber("Angle", getAngle());
        SmartDashboard.putNumber("Motor Out", motorOut);
        SmartDashboard.putNumber("goal angle", getGoalRoatation());
  
    }
    public boolean isLowestAngle(){
        return !_lowButton.get();
    }
    public boolean isHighestAngle(){
        return !_highButton.get() || getEncoderPosition() < ARM_MAX_ANGLE || getEncoderPosition() > 0.3;
    }
    public double getAngle(){
        return _encoder.getAbsolutePosition().getValueAsDouble();
    }
    public void calibrate(){
        if(isLowestAngle()){
            _motor.stopMotor();
          
        }
    }
    public void setArmRotation(double angle){
        goalAngle = angle;
    }
    public double getGoalRoatation(){
        return goalAngle;
    }
    public void rotateArmAngle(double amount) {
        if((amount < 0 && !isHighestAngle()) || (amount > 0 && !isLowestAngle())) {
            goalAngle = Math.min(Math.max(goalAngle + amount, ARM_MAX_ANGLE), ARM_INTAKE_ANGLE);
        }
    }
    public void setAngle(double angle){
       setArmRotation(angle);
       motorOut = Math.max(Math.min(_angleController.calculate(getEncoderPosition(), getGoalRoatation()), 1), -1);
    if (isLowestAngle() && motorOut > 0) {
        setArmRotation(getEncoderPosition());
        motorOut = 0;
    }

    if ((isHighestAngle()) && motorOut < 0) {
        setArmRotation(getEncoderPosition());
        motorOut = 0;
    }
    _motor.set(motorOut);
    
    }
    
    public double getEncoderPosition(){
        return _encoder.getAbsolutePosition().getValueAsDouble();
    }
    public void setMotorDirect(double speed){
        _motor.set(speed);
    }


    
    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        SmartDashboard.putBoolean("Is Lowest", isLowestAngle());
        SmartDashboard.putBoolean("Is Highest", isHighestAngle());

        SmartDashboard.putNumber("Angle", getAngle());
        SmartDashboard.putNumber("Motor Out", motorOut);
        SmartDashboard.putNumber("goal angle", getGoalRoatation());

        super.periodic();
    }


    
}

