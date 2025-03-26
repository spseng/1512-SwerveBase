package frc.robot.commands.EndEffector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Elevator;
import frc.robot.Utils.Helpers;
import frc.robot.Utils.Vector2d;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EndEffector;

public class IntakeWheeels extends Command{
   
    private final EndEffector _endEffector;

    public IntakeWheeels( EndEffector endEffector){
        
        _endEffector = endEffector;
    }
    
    @Override
    public void execute() {
        _endEffector.setIntakeSpeed(-1);
        
    }
}
