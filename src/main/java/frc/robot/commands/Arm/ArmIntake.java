package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmIntake extends Command{

    private final Arm _arm;
    private final Elevator _elevator;

    public ArmIntake(Arm arm, Elevator elevator){

        _arm = arm;
        _elevator = elevator;
    }

    @Override
    public void execute() {
        if (_elevator.getCurrentHeight() > Constants.Elevator.L1_HEIGHT){
            _elevator.setTargetHeight(Constants.Elevator.INTAKE_HEIGHT);
        } else {
            _arm.setArmPosition(Constants.Arm.ARM_INTAKE_ANGLE);
        }
     _arm.setArmPosition(Constants.Arm.L4_ANGLE);

    }
    
    
}
