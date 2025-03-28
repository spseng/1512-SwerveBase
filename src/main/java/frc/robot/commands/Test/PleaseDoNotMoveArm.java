package frc.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class PleaseDoNotMoveArm extends Command {
    private final Arm _arm;
    public PleaseDoNotMoveArm(Arm arm) {
        _arm = arm;
        addRequirements(_arm);
        // Please do not move the elevator
    }

    @Override
    public void execute() {
        _arm.setArmPosition(_arm.getCurrentAngle());
    }
}
