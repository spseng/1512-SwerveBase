package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class EndEffector extends SubsystemBase{

    private SparkMax _topMotor;
    private SparkMax _bottomMotor;
    private DigitalInput _proxsensor;

    public EndEffector(){
        _topMotor = new SparkMax(RobotMap.CAN.ENDEFFECTOR_MOTOR_TOP_CAN, MotorType.kBrushed);
        _bottomMotor = new SparkMax(RobotMap.CAN.ENDEFFECTOR_MOTOR_BOTTOM_CAN, MotorType.kBrushed);
        _proxsensor = new DigitalInput(RobotMap.DIO.ENDEFFECTOR_PROX_SENSOR_ID);

    } 
    public boolean isCoralInIntake(){
        return _proxsensor.get();
    }
    public void setIntakeSpeed(double speed){
        _topMotor.set(speed);
        _bottomMotor.set(-speed);
    }
}
