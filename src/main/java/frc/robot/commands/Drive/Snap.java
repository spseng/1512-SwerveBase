package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;


public class Snap extends Command  {

    private final Drivetrain _drivetrain;
    private final Rotation2d _heading;
    


    public Snap(Drivetrain drivetrain, Rotation2d heading){

        _drivetrain = drivetrain;
        _heading = heading;
        



    }

   
    @Override
    public void execute() {
       _drivetrain.setHeadingRotation2D(_heading);
    }

    
}
