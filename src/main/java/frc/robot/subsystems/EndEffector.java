package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase{

    private SparkMax _motor;
    private DigitalInput _proxsensor;

    public EndEffector(){
        _motor = new SparkMax(0, MotorType.kBrushless);
        _proxsensor = new DigitalInput(0);

    } 
    public boolean isCoralInIntake(){
        return _proxsensor.get();
    }
    public void setIntakeSpeed(double speed){
        _motor.set(speed);
    }
}
