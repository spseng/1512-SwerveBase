package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.EndEffector;

public class IntakeCoralFunnel extends Command {
    private final EndEffector _endEffector;
    private final Arm _arm;

    public IntakeCoralFunnel(EndEffector endEffector, Arm arm){

        _endEffector = endEffector;
        _arm = arm;
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
        _arm.setArmPosition(Constants.Arm.ARM_INTAKE_ANGLE);
        _endEffector.setIntakeSpeed(Constants.In.INTAKE_SPEED);

    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return _endEffector.isCoralInIntake();
    }
    
    
    
}
