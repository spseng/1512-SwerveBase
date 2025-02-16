package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
    private final SparkMax elevatorMotorLeft;
    private final SparkMax elevatorMotorRight;

    private final SparkMaxConfig elevatorMotorLeftConfig;
    private final SparkMaxConfig elevatorMotorRightConfig;

    private final AbsoluteEncoder elevatorEncoder;

    private final PIDController elevatorPIDController;
    public Elevator() {
        elevatorMotorLeft = new SparkMax(RobotMap.CAN.ELEVATOR_MOTOR_LEFT, MotorType.kBrushless);
        elevatorMotorRight = new SparkMax(RobotMap.CAN.ELEVATOR_MOTOR_RIGHT, MotorType.kBrushless);

        elevatorMotorLeftConfig = new SparkMaxConfig();
        elevatorMotorRightConfig = new SparkMaxConfig();

        elevatorEncoder = elevatorMotorLeft.getAbsoluteEncoder();

        elevatorPIDController = new PIDController(Constants.Elevator.ELEVATOR_KP, Constants.Elevator.ELEVATOR_KI, Constants.Elevator.ELEVATOR_KD);
    }
}
