package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
    private final CANSparkMax _motor;
    private final DigitalInput _intakeButton;



    public Intake(){
        _motor = new CANSparkMax(RobotMap.CAN.INTAKE_MOTOR_CAN, MotorType.kBrushless);
        _intakeButton = new DigitalInput(RobotMap.DIO.SHOOTER_IS_LOADED_BUTTON_ID);
    }
    public void setIntakeSpeed(double speed){
        _motor.set(speed);
    }
    public void setIntakeSpeedTimed(long time, double speed){
    long timeout = (System.currentTimeMillis() + time);
    setIntakeSpeed(speed);
        if (timeout <= System.currentTimeMillis()) {
        StopIntake();
        }
    }
    public boolean isInIntake(){
        return _intakeButton.get();
    }
    public boolean isIntaking(){
        return (_motor.get() != 0);
    }
    public void StopIntake(){
        _motor.set(0);
    }
     public void UpdateDashboard(){
        SmartDashboard.putBoolean("Is Intaking", isIntaking());
        SmartDashboard.putBoolean("Is in Intake", isInIntake());
     }



}