package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class IdleArm extends Command {

    private final Arm _arm;

    public IdleArm(Arm arm){
        _arm = arm;
        addRequirements(_arm);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        //_arm.setArmPosition(_arm.getAngle());
    }
    
}
