package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class SetIntakeAngle extends Command{
    private Arm _arm;
    public SetIntakeAngle(Arm arm){
        _arm = arm;
        addRequirements(_arm);

    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        _arm.setAngle(Constants.Arm.ARM_INTAKE_ANGLE);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return (Constants.Arm.ARM_INTAKE_ANGLE - Constants.Arm.TOLERANCE < _arm.getAngle() || Constants.Arm.ARM_INTAKE_ANGLE + Constants.Arm.TOLERANCE > _arm.getAngle() || _arm.isLowestAngle());
    }
    
}

