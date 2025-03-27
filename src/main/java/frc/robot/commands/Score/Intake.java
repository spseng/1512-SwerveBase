package frc.robot.commands.Score;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.EndEffectorIntake;
import frc.robot.commands.Elevator.ElevatorIntake;
import frc.robot.commands.EndEffector.ArmIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class Intake extends SequentialCommandGroup {

    private final Elevator _elevator;
  
    private final Arm _arm;
    private final EndEffector _endEffector;

    
    public Intake(Elevator elevator, Arm arm, EndEffector endEffector){

        _elevator = elevator;
        _arm = arm;
        _endEffector = endEffector;

        addCommands(
            new ElevatorIntake(_elevator, _arm),
            new WaitCommand(.5),
            new ArmIntake(arm),
            new EndEffectorIntake(endEffector)

        );

    }
    
    
    
    
}
