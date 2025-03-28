package frc.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class PleaseDoNotMoveElevator extends Command {
    private final Elevator _elevator;
    public PleaseDoNotMoveElevator(Elevator elevator) {
        _elevator = elevator;
        addRequirements(_elevator);
        // Please do not move the elevator
    }

    @Override
    public void initialize() {
        _elevator.setTargetHeight(_elevator.getCurrentHeight());
    }
}
