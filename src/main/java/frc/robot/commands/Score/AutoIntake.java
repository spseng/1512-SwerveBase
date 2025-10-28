package frc.robot.commands.Score;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.EndEffectorIntake;
import frc.robot.commands.WaitForIntake;
import frc.robot.commands.EndEffector.StopWheels;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffectorCTR;

public class AutoIntake extends SequentialCommandGroup {

    private EndEffectorCTR _endEffector;
    private Elevator _elevator;
    private Arm _arm;

    public AutoIntake(EndEffectorCTR endEffector){

        _endEffector = endEffector;

        addCommands(
            new EndEffectorIntake(_endEffector),
            new WaitForIntake(_endEffector),
            new WaitCommand(.2),
            new StopWheels(_endEffector)
        );
    }
    
}
