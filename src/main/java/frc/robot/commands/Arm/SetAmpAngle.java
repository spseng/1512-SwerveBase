package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class SetAmpAngle extends Command{
    private final Arm _arm;
    public SetAmpAngle(Arm arm){
        _arm = arm;
        addRequirements(_arm);

    }
    @Override
    public void initialize() {
        super.initialize();
    }
    @Override
    public void execute() {
        super.execute();
        _arm.setArmPosition(Constants.Arm.AMP_SCORING_ANGLE);
    }
    @Override
    public boolean isFinished() {
        return (Constants.Arm.AMP_SCORING_ANGLE - Constants.Arm.TOLLERENCE < _arm.getAngle() || Constants.Arm.AMP_SCORING_ANGLE + Constants.Arm.TOLLERENCE > _arm.getAngle());
    }
    
}
