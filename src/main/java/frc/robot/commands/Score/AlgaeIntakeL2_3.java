package frc.robot.commands.Score;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class AlgaeIntakeL2_3 {
    private final Elevator _elevator;
    private final Arm _arm;
    private boolean colision = false;

    AlgaeIntakeL2_3(Elevator elevator, Arm arm) {
        _elevator = elevator;
        _arm = arm;
    }

    public void initialize() {
        colision = _arm.isColision();
    }

    public void execute() {
        // TODO Auto-generated method stub
        if (colision) {
            _arm.setArmPosition(Constants.Arm.ARM_SAFE_ANGLE);
        } else {
            _elevator.setTargetHeight(Constants.Elevator.ALGAE_L2_3_HEIGHT);
            _arm.setArmPosition(Constants.Arm.ALGAE_L2_3_ANGLE);
        }
    }

    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
}
