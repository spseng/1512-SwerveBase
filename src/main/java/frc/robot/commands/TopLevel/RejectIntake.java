package frc.robot.commands.TopLevel;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.In.*;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class RejectIntake extends Command {
    private final Intake _intake;
    private final Indexer _indexer;
    private final Shooter _shooter;

    public RejectIntake(Intake intake, Indexer indexer, Shooter shooter){
        _shooter = shooter;
        _indexer = indexer;
        _intake = intake;
        addRequirements(_indexer, _intake, _intake);

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
        
        _intake.setIntakeSpeed(Constants.In.EJECT_SPEED);
        _indexer.setSpeed(Constants.In.EJECT_SPEED);
        _shooter.setSpeed(Constants.In.EJECT_SPEED);
     
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
    
}

