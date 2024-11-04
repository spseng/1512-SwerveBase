package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Utils.AxisButton;
import frc.robot.Utils.Gamepad;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Utils.Helpers.*;
import frc.robot.commands.SimpleIntake;
import frc.robot.commands.Arm.ClimbUp;
import frc.robot.commands.Arm.IncrementDown;
import frc.robot.commands.Arm.IncrementUp;
import frc.robot.commands.Arm.SetAmpAngle;
import frc.robot.commands.Arm.SetIntakeAngle;
import frc.robot.commands.Drive.ResetIMU;
import frc.robot.commands.TopLevel.IntakeNote;
import frc.robot.commands.TopLevel.RejectIntake;
import frc.robot.commands.TopLevel.ShootAmp;
import frc.robot.commands.TopLevel.ShootWoofer;

public class OI {
    
    protected Gamepad _driverGamepad;
  

    protected AxisButton _driverLeftTriggerButton;
    protected AxisButton _driverRightTriggerButton;

    protected Trigger _driverLeftTrigger;
    protected Trigger _driverRightTrigger;
    protected Trigger _povButtonLeft;
    protected Trigger _povButtonRight;
    protected Trigger _povButtonUp;
    protected Trigger _povButtonDown;
    protected Trigger _opPovButtonDown;
    protected Trigger _opPovButtonRight;
    protected Trigger _opPovButtonUp;
    protected Trigger _opPovButtonLeft;

public OI(){

    _driverGamepad = new Gamepad(0);
   

    _povButtonLeft = new Trigger(() -> _driverGamepad.getPOV() == 270);
    _povButtonRight = new Trigger(() -> _driverGamepad.getPOV() == 90);
    _povButtonUp = new Trigger(() -> _driverGamepad.getPOV() == 0);
    _povButtonDown = new Trigger(() -> _driverGamepad.getPOV() == 180);
 

    _driverLeftTriggerButton = new AxisButton(_driverGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), 0.05);
    _driverLeftTrigger = new Trigger(_driverLeftTriggerButton::get);

    _driverRightTriggerButton = new AxisButton(_driverGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), 0.05);
    _driverRightTrigger = new Trigger(_driverRightTriggerButton::get);
}

public void initializeButtons(
    Drivetrain drivetrain,
    Shooter shooter,
    Indexer indexer,
    Intake intake,
    Arm arm
){

    _driverLeftTrigger.whileTrue(new SimpleIntake(intake, indexer));
    _driverRightTrigger.whileTrue(new ShootWoofer(arm, shooter, indexer));
    //this is where we map commands

   _driverGamepad.getLeftBumper().whileTrue(new RejectIntake(intake, indexer, shooter));
   _driverGamepad.getRightBumper().whileTrue(new SetAmpAngle(arm));
   _driverGamepad.getXButton().onTrue(new SetIntakeAngle(arm));
   _driverGamepad.getYButton().whileTrue( new IncrementUp(arm));
   _driverGamepad.getAButton().whileTrue(new IncrementDown(arm));
   _driverGamepad.getBButton().onTrue(new ResetIMU(drivetrain));
   
    

}
public double getDriveY() {
    double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
    speed = applyDeadband(speed, Constants.Drivetrain.TRANSLATION_DEADBAND);
    return speed;
}

public double getDriveX() {
    double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber());
    speed = applyDeadband(speed, Constants.Drivetrain.TRANSLATION_DEADBAND);
    return speed;
}

public double getRotationX() {
    double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_X.getNumber());
    speed = applyDeadband(speed, Constants.Drivetrain.ROTATION_DEADBAND);
    return speed;
}
public static double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
        if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
        } else {
            return (value + deadband) / (1.0 - deadband);
        }
    } else {
        return 0.0;
    }
}


protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

public void rumbleDriver() {
        _driverGamepad.setRumble(GenericHID.RumbleType.kBothRumble, 1);
}

public void stopRumbleDriver() {
        _driverGamepad.setRumble(GenericHID.RumbleType.kBothRumble, 0);

}


}