package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
    private CANSparkMax _motor;

    public Shooter(){
        _motor = new CANSparkMax(RobotMap.CAN.SHOOTER_MOTOR_CAN, MotorType.kBrushless);
    }
    public void setSpeed(double speed){
        _motor.set(speed);
    }
    public double getSpeed(){
        return _motor.get();
    }
    public void UpdateDashboard(){
        SmartDashboard.putNumber("Shooter Set Speed", getSpeed());
    }
    
}
