package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{

    private final SparkMax _motor;
    private final SparkAbsoluteEncoder _encoder;
    private final SparkMaxConfig _config;

    private double _setpoint;

    private final SparkClosedLoopController _controller;

    
    public Arm(){
        _motor = new SparkMax(0, MotorType.kBrushed); //TODO fillin IDs
        _encoder = _motor.getAbsoluteEncoder();
        _controller = _motor.getClosedLoopController();
        _config = new SparkMaxConfig();
        _config.closedLoop
            .pid(0,0,0) //TODO fill in
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .positionWrappingEnabled(false)
            .outputRange(0,0);

        _config.absoluteEncoder
            .inverted(false); //TODO add conversion factors if needed. 


    }
    public double getArmAngle(){
        return _encoder.getPosition(); // Will Need to be Scaled
    }
    public void setArmPosition(double position){
        _setpoint = position;
    }
    public double getSetpoint(){
        return _setpoint;
    }
    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        _controller.setReference(_setpoint, ControlType.kPosition);
        updateSmartDashboard();
        
    }
    public void updateSmartDashboard(){
        SmartDashboard.putNumber("Arm Setpoint", _setpoint);
        SmartDashboard.putNumber("Arm Position", _encoder.getPosition());
    }
}
