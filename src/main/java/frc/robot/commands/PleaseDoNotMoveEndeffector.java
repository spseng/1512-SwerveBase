package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Elevator;

public class PleaseDoNotMoveEndeffector extends Command {
    private final EndEffector _endEffector;
    public PleaseDoNotMoveEndeffector(EndEffector endEffector) {
        _endEffector = endEffector;
        addRequirements(_endEffector);
        // Please do not move the elevator
    }

    @Override
    public void execute() {
        _endEffector.setIntakeSpeed(0.0);
    }
}
