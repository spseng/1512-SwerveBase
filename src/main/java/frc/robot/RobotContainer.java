// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IdleIndexer;
import frc.robot.commands.IdleIntake;
import frc.robot.commands.IdleShooter;
import frc.robot.commands.Arm.IdleArm;
import frc.robot.commands.Drive.Drive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.commands.IdleIndexer.*;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 private Drivetrain _drivetrain;
  private Intake _intake;
  private Shooter _shooter;
  private Indexer _indexer;
  private Robot _robot;
  private Arm _arm;
  private AHRS _gyro;

  private OI _oi;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //add controler in OI
      

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Robot robot) {
    // Configure the trigger bindings
  _robot = robot;
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
  public void init(){
    _indexer = new Indexer();
    _drivetrain = new Drivetrain();
    _shooter = new Shooter();
    _intake = new Intake();
    _arm = new Arm();
    _oi = new OI();

    _arm.setDefaultCommand(new IdleArm(_arm));
    _drivetrain.setDefaultCommand(new Drive(_oi, _drivetrain));
    _shooter.setDefaultCommand(new IdleShooter(_shooter));
    _indexer.setDefaultCommand( new IdleIndexer(_indexer));
    _intake.setDefaultCommand( new IdleIntake(_intake));

    _oi.initializeButtons(_drivetrain, _shooter, _indexer, _intake, _arm);
    

  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Command() {
      
    };
  }
   private void setDefaultCommand(SubsystemBase subSystem, Command command) {
        if (subSystem == null || command == null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }
}
