package frc.robot.commands.Score;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.EndEffectorIntake;
import frc.robot.commands.Arm.ArmL2;
import frc.robot.commands.Elevator.ElevatorIntake;
import frc.robot.commands.Elevator.ElevatorL2;
import frc.robot.commands.EndEffector.ArmIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class ScoreL3 extends SequentialCommandGroup {

    private final Elevator _elevator;
  
    private final Arm _arm;
    

    
    public ScoreL3(Elevator elevator, Arm arm){

        _elevator = elevator;
        _arm = arm;
       

        addCommands(
            new ElevatorL2(_elevator, _arm),
            new WaitCommand(.5),
            new ArmL2(arm)
           

        );

    }
}
    
    