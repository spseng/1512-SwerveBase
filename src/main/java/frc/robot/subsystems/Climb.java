package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase{

    private SparkMax _motor;
    private DigitalInput _limitswitch;

    public Climb(){
        _motor = new SparkMax(RobotMap.CAN.CLIMB_MOTOR_CAN, MotorType.kBrushless);
        _limitswitch = new DigitalInput(1);
    }

    public void setClimbSpeed(double speed){
        _motor.set(speed);
    }

    public void stop() {
        _motor.set(0);
    }    
    public boolean isDown(){
        return _limitswitch.get();
    }

}
