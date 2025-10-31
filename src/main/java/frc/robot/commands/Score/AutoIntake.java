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

    public AutoIntake(EndEffectorCTR endEffector, Arm arm, Elevator elevator){

        _endEffector = endEffector;
        _arm = arm;
        _elevator = elevator;

        addCommands(
            new HalfDownIntake(_elevator, _arm, endEffector),
            new FullDownIntake(_elevator, _arm, endEffector),
            new WaitForIntake(_endEffector),
            new WaitCommand(.04),
            new StopWheels(_endEffector)
        );
    }
    
}
