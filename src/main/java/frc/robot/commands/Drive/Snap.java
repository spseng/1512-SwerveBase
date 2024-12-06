package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;


public class Snap extends Command  {

    private final Drivetrain _drivetrain;
    private final OI _oi;
    


    public Snap(Drivetrain drivetrain, OI oi ){

        _drivetrain = drivetrain;
        _oi =oi;



    }

   
    @Override
    public void execute() {
       _drivetrain.setHeadingRotation2D(new Rotation2d(_oi.getSnapHeading()));
    }

    
}
