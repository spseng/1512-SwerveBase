package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj.Timer;

public class PlaceL1 extends Command {

    private final EndEffector _endEffector;
    private final Arm _arm;
    private final Elevator _elevator;

    private double startSpinTime;
    private boolean isSpinning;
    private static final double SPIN_DURATION = 4.0;
    
    public PlaceL1(EndEffector endEffector, Arm arm, Elevator elevator){
        _endEffector = endEffector;
        _arm = arm;
        _elevator = elevator;


    }
    @Override
    public void initialize() {
        _arm.setArmPosition(Constants.Arm.L1_ANGLE); 
        _elevator.setTargetHeight(Constants.Elevator.L1_HEIGHT); 
        isSpinning = false;
        startSpinTime = 0.0;
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        if (_elevator.isAtTarget() && _arm.isAtTarget()) {
            if (!isSpinning) {
                startSpinTime = Timer.getFPGATimestamp();
                isSpinning = true;
            }
            _endEffector.setIntakeSpeed(Constants.EndEffector.OUTTAKE_SPEED);
        }
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        if (isSpinning && (Timer.getFPGATimestamp() - startSpinTime >= SPIN_DURATION)) {
            return true;
        }
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
    
}
