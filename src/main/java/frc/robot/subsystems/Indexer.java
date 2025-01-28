package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Indexer extends SubsystemBase{

    private SparkMax _motor;

    public Indexer(){
        _motor = new SparkMax(RobotMap.CAN.LOADING_MOTOR_CAN, MotorType.kBrushless);
    }
    public void setSpeed(double speed){
        _motor.set(speed);
    }
    public double getSpeed(){
        return _motor.get();
    }
    public void UpdateDashboard(){
        SmartDashboard.putNumber("Indexer Speed", getSpeed());
    }
    public void setIndexerSpeedTimed(long time, double speed){
        long timeout = (System.currentTimeMillis() + time);
        setSpeed(speed);
            if (timeout <= System.currentTimeMillis()) {
            StopIndexer();
            }
        }
        public void setIndexerSpeedAfterTime(long time, double speed){
        long timeout = (System.currentTimeMillis() + time);
            StopIndexer();
            if (timeout <= System.currentTimeMillis()) {
            setSpeed(speed);
            }
        }
    public void StopIndexer(){
        _motor.set(0);
    }
    
}
