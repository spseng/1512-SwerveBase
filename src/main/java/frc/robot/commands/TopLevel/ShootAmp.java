package frc.robot.commands.TopLevel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends Command {

    private final Indexer _indexer;
    private final Shooter _shooter;
    private long timeout;
    public ShootAmp(Indexer indexer, Shooter shooter){
        _indexer = indexer;
        _shooter = shooter;
        addRequirements(_shooter, _indexer);

    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        timeout = System.currentTimeMillis() + Constants.Shooter.AMP_SCORE_TIME;
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        _shooter.setSpeed(Constants.Shooter.SHOOTER_AMP_SPEED);
        _indexer.setIndexerSpeedAfterTime(Constants.In.SHOOT_DELAY, Constants.In.SHOOT_STATIC_SPEED);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return (timeout < System.currentTimeMillis());
    }
    
}
