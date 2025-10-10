package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import com.ctre.phoenix6.hardware.TalonFX;

public class EndEffectorCTR extends SubsystemBase{

    private TalonFX _motor;
    private DigitalInput _proxsensor;

    public EndEffectorCTR(){
        _motor = new TalonFX(RobotMap.CAN.ENDEFFECTOR_MOTOR_TOP_CAN);
        _proxsensor = new DigitalInput(RobotMap.DIO.ENDEFFECTOR_PROX_SENSOR_ID);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral in Intake", isCoralInIntake());
    }

    public boolean isCoralInIntake(){
        return !_proxsensor.get();
    }

    public void setIntakeSpeed(double speed){
        _motor.set(speed);
    }

    public void stop() {
        _motor.set(0);
    }
}
