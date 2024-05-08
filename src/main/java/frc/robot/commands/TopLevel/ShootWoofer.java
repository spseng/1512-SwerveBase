package frc.robot.commands.TopLevel;

import static frc.robot.Constants.Arm.ARM_INTAKE_ANGLE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShootWoofer extends Command{

    private final Shooter _shooter;
    private final Indexer _indexer;
    private final Arm _arm;

    private long timeout;

    public ShootWoofer(Arm arm, Shooter shooter, Indexer indexer){
        _arm = arm;
        _shooter = shooter;
        _indexer = indexer;
        addRequirements(_shooter, _arm, _indexer);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        _arm.setAngle(Constants.Arm.SHOOT_SUB_ANGLE);
        _shooter.setSpeed(Constants.Shooter.SHOOTER_SUBWOOFER_SPEED);
        timeout = System.currentTimeMillis() + Constants.Shooter.SHOOT_WOOF_DELAY;
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        _indexer.setIndexerSpeedAfterTime(Constants.In.SHOOT_DELAY, Constants.In.SHOOT_STATIC_SPEED);

    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return (timeout < System.currentTimeMillis());
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        _arm.setAngle(Constants.Arm.ARM_INTAKE_ANGLE);
        super.end(interrupted);
    }
    
}
