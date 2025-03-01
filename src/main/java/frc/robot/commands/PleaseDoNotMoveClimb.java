package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;

public class PleaseDoNotMoveClimb extends Command {
    private final Climb _climb;
    public PleaseDoNotMoveClimb(Climb climb) {
        _climb = climb;
        addRequirements(_climb);
        // Please do not move the elevator
    }

    @Override
    public void execute() {
        _climb.setClimbSpeed(0.0);
    }
}
