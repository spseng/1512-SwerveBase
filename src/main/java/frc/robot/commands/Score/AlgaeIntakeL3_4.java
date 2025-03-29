package frc.robot.commands.Score;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class AlgaeIntakeL3_4 extends Command {
    private final Elevator _elevator;
    private final Arm _arm;
    private boolean colision = false;

    public AlgaeIntakeL3_4(Arm arm, Elevator elevator) {
        _elevator = elevator;
        _arm = arm;
    }

    @Override
    public void initialize() {
        colision = _arm.isColision();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        if (colision) {
            _arm.setArmPosition(Constants.Arm.ARM_SAFE_ANGLE);
        } else {
            _elevator.setTargetHeight(Constants.Elevator.ALGAE_L2_3_HEIGHT);
            _arm.setArmPosition(Constants.Arm.ALGAE_L2_3_ANGLE);
        }
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
}
