package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase{

    private SparkMax _motor;

    public Climb(){
        _motor = new SparkMax(RobotMap.CAN.CLIMB_MOTOR_CAN, MotorType.kBrushless);
    }

    public void setClimbSpeed(double speed){
        _motor.set(speed);
    }
}
