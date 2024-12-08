package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IdleIntake extends Command {

    private final Intake _intake;
    public IdleIntake(Intake intake){
      _intake = intake;
      addRequirements(_intake);
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        _intake.setIntakeSpeed(0);
    }
    
}

