package frc.robot.commands.Score;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.EndEffectorIntake;
import frc.robot.commands.WaitForIntake;
import frc.robot.commands.EndEffector.StopWheels;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffectorCTR;

public class AutoIntake extends SequentialCommandGroup {

    private EndEffectorCTR _endEffector;

    public AutoIntake(EndEffectorCTR endEffector){

        _endEffector = endEffector;

        addCommands(
            new EndEffectorIntake(_endEffector),
            new WaitForIntake(_endEffector),
            new WaitCommand(.4),
            new StopWheels(endEffector)
        );
    }
    
}
