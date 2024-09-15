package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Utils.AxisButton;
import frc.robot.Utils.Gamepad;
import frc.robot.Utils.Helpers;

import frc.robot.commands.Drive.ResetIMU;
import frc.robot.commands.Drive.Snap;
import frc.robot.subsystems.Drivetrain;


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
    Drivetrain drivetrain
   
){


    //this is where we map commands

   //_driverGamepad.getAButton().onTrue(new ShootAmp(indexer, shooter));
   //_driverGamepad.getYButton().onTrue(new ClimbUp(arm));
   _driverGamepad.getBButton().onTrue(new ResetIMU(drivetrain));
   _povButtonDown.onTrue(new Snap(drivetrain, this));
   _povButtonLeft.onTrue(new Snap(drivetrain, this));
   _povButtonUp.onTrue(new Snap(drivetrain, this));
   _povButtonRight.onTrue(new Snap(drivetrain, this));
 
    

}
public double getDriveY() {
    double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
    speed = Helpers.applyDeadband(speed, Constants.Drivetrain.TRANSLATION_DEADBAND);
    return speed;
}

public double getDriveX() {
    double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber());
    speed = Helpers.applyDeadband(speed, Constants.Drivetrain.TRANSLATION_DEADBAND);
    return speed;
}

public double getRotationX() {
    double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_X.getNumber());
    speed = Helpers.applyDeadband(speed, Constants.Drivetrain.ROTATION_DEADBAND);

    return speed;
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
public boolean getArmUp(){
    return _driverGamepad.getYButton().getAsBoolean();
}
public boolean getArmDown(){
    return _driverGamepad.getAButton().getAsBoolean();
}
public double getSnapHeading(){
    double heading;
    if (_driverGamepad.povUp(null).getAsBoolean()){
        heading = 0.0;
    } else if (_driverGamepad.povDown(null).getAsBoolean()){
        heading = 180.0;
    } else if (_driverGamepad.povLeft(null).getAsBoolean()){
        heading = 90.0;
    } else if (_driverGamepad.povLeft(null).getAsBoolean()){
        heading = 270.0;
    } else {
        heading = 0.0;
    }
    return heading;
}



}