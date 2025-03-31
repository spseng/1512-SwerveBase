package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class WaitForIntake extends Command {

    private EndEffector _endEffector;

    public WaitForIntake(EndEffector endEffector){
        _endEffector = endEffector;

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
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return _endEffector.isCoralInIntake();
    }
    
}
