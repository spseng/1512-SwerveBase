// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Utils.Vision.VisionProcessor;
//import frc.robot.commands.Autonomous.AutonomousScoreApproach;
//import frc.robot.commands.Autonomous.MoveALittleBit;
import frc.robot.commands.Drive.Drive;
import frc.robot.commands.Test.ArmTest;
import frc.robot.commands.Test.ClimbTest;
import frc.robot.commands.Test.ElevatorTest;
import frc.robot.commands.Test.EndeffectorTest;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Drivetrain;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    //private VisionProcessor _visionProcessor;
    //private Drivetrain _drivetrain;
    //private VisionProcessor _visionProcessor;
    private Drivetrain _drivetrain;
    private OI _oi;
    //private AutonomousConfigure _autonomous;
    private Elevator _elevator;
    private Arm _arm;
    private EndEffector _endEffector;
    private Climb _climb;
    // Replace with CommandPS4Controller or CommandJoystick if needed
    //add controller in OI

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */

    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
    }

    public void init() {
        //_visionProcessor = new VisionProcessor();
        _drivetrain = new Drivetrain();
        _elevator = new Elevator();
        _arm = new Arm();
        _endEffector = new EndEffector();
        _climb = new Climb();
        _oi = new OI(_drivetrain, _elevator, _arm, _endEffector);
        //_autonomous = new AutonomousConfigure();
        //_drivetrain.setDefaultCommand(new AutonomousScoreApproach(_drivetrain, "camera2"));
       

        //_elevator.setDefaultCommand(new PleaseDoNotMoveElevator(_elevator));
        //_arm.setDefaultCommand(new PleaseDoNotMoveArm(_arm));
        //_endEffector.setDefaultCommand(new PleaseDoNotMoveEndeffector(_endEffector));
        //_climb.setDefaultCommand(new PleaseDoNotMoveClimb(_climb));

        //_drivetrain.setDefaultCommand(new Drive(_oi, _drivetrain));
        //_elevator.setDefaultCommand(new ElevatorTest(_oi, _elevator));
        //_arm.setDefaultCommand(new ArmTest(_oi, _arm));
        _climb.setDefaultCommand(new ClimbTest(_oi, _climb));
        //_endEffector.setDefaultCommand(new EndeffectorTest(_oi, _endEffector));

        _oi.initializeButtons();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        //return new MoveALittleBit(_drivetrain).withTimeout(10);
        return Commands.none();
        // An ExampleCommand will run in autonomous
        /*
        /*
        try {
            return AutoBuilder.followPath(_autonomous.getSelectedPath());
        }catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            SmartDashboard.putString("Auto Error", e.getMessage());
            return Commands.none();
        }
        */
    }
}