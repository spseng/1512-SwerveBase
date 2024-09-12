package frc.robot.subsystems;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Utils.SwerveHeadingController;
import frc.robot.Utils.SwerveSetpoint;
import frc.robot.Utils.SwerveSetpointGenerator;
import frc.robot.Utils.SwerveSetpointGenerator.KinematicLimits;

public class Drivetrain extends SubsystemBase {

    public static class SystemIO { // this is a wrapper for all inputs and output to drive
        ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0); // chassis speeds is a velocity in x & y and rotation in radians per seonds
        SwerveModuleState[] measuredStates = new SwerveModuleState[] { // state array to control modules
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };
        SwerveModulePosition[] measuredPositions = new SwerveModulePosition[] { // position is robot relative
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };
        Rotation2d heading = new Rotation2d(0.0);

        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[4]); // cheesy stuff 
    }

    private SwerveModule[] _modules;

    private static final int NORTH_EAST_IDX = 0;
    private static final int NORTH_WEST_IDX = 1;
    private static final int SOUTH_EAST_IDX = 2;
    private static final int SOUTH_WEST_IDX = 3;

    private final SwerveSetpointGenerator _setpointGenerator;
    private final KinematicLimits _limits;
    private final SwerveDriveKinematics _kinematics; // physical layout of chassis

    private double _yawoffset;

    private SwerveHeadingController _heading;

    private SystemIO _Io;

    private final AHRS _gyro; // navX might want to swit ch to pigeon 2








    public Drivetrain(){
        _Io = new SystemIO();
        _gyro = new AHRS(SPI.Port.kMXP); // I think that this is right

        _yawoffset = _gyro.getYaw(); // zero gyro on init
        readIMU(); // method to update gyro
        
        _modules = new SwerveModule[4];

        _modules[NORTH_WEST_IDX] = new SwerveModule(RobotMap.CAN.FL_DRIVE_CAN, RobotMap.CAN.FL_STEER_CAN, Constants.Drivetrain.NORTH_WEST_CONFIG); // TODO CHANGUS
        _modules[NORTH_EAST_IDX] = new SwerveModule(RobotMap.CAN.FR_DRIVE_CAN, RobotMap.CAN.FR_STEER_CAN, Constants.Drivetrain.NORTH_EAST_CONFIG); // TODO CHANGUS
        _modules[SOUTH_WEST_IDX] = new SwerveModule(RobotMap.CAN.BL_DRIVE_CAN, RobotMap.CAN.BL_STEER_CAN, Constants.Drivetrain.SOUTH_WEST_CONFIG); // TODO CHANGUS
        _modules[SOUTH_EAST_IDX] = new SwerveModule(RobotMap.CAN.BR_DRIVE_CAN, RobotMap.CAN.BR_STEER_CAN, Constants.Drivetrain.SOUTH_EAST_CONFIG); // TODO CHANGUS

        _kinematics = new SwerveDriveKinematics( //location in where it is on chassis
            _modules[NORTH_EAST_IDX].getSwerveModuleLocation(),
            _modules[NORTH_WEST_IDX].getSwerveModuleLocation(),
            _modules[SOUTH_EAST_IDX].getSwerveModuleLocation(),
            _modules[SOUTH_WEST_IDX].getSwerveModuleLocation()
        );
        _setpointGenerator = new SwerveSetpointGenerator(
                _kinematics,
                new Translation2d[] {
                        _modules[NORTH_EAST_IDX].getSwerveModuleLocation(),
                        _modules[NORTH_WEST_IDX].getSwerveModuleLocation(),
                        _modules[SOUTH_EAST_IDX].getSwerveModuleLocation(),
                        _modules[SOUTH_WEST_IDX].getSwerveModuleLocation()
                      
                }); // chessy stuff
        _heading = new SwerveHeadingController(0.2);     // not sure if we want to use this
        _limits = Constants.Drivetrain.DRIVE_KINEMATIC_LIMITS;
        ZeroIMU(); // resets heading
        readModules(); // gets encoders 
        setSetpointFromMeasuredModules(); // cheesy stuff
        }

        public void setRawChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] desiredModuleState = _kinematics.toSwerveModuleStates(speeds);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleState, Constants.Drivetrain.MAX_TURING_SPEED); // TODO CHanGUS
        _Io.setpoint.moduleStates = desiredModuleState; // desaturate wheel speeds makes it so that no wheel is required to do more than it can.
    }

    @Override
    public void periodic() {
        readIMU(); // gets data from IMU
        readModules(); // gets Encoder Data
        updateShuffleBoard(); // log
        updateDesiredStates(); // 
        setModuleStates(_Io.setpoint.moduleStates); // sets modules new desired states
       

    }
    public SwerveSetpoint getSetpoint() {
        return _Io.setpoint;
    }
    public void updateDesiredStates() {
        // This is to avoid skew when driving and rotating.
        Pose2d robotPoseVel = new Pose2d(
                _Io.desiredChassisSpeeds.vxMetersPerSecond * Constants.UPDATE_PERIOD,
                _Io.desiredChassisSpeeds.vyMetersPerSecond * Constants.UPDATE_PERIOD,
                Rotation2d.fromRadians(
                        _Io.desiredChassisSpeeds.omegaRadiansPerSecond * Constants.UPDATE_PERIOD));

        Twist2d twistVel = new Pose2d().log(robotPoseVel);

        ChassisSpeeds updatedChassisSpeeds = new ChassisSpeeds(
                twistVel.dx / Constants.UPDATE_PERIOD,
                twistVel.dy / Constants.UPDATE_PERIOD,
                twistVel.dtheta / Constants.UPDATE_PERIOD); // all of this is to conserve heading

        _Io.setpoint = _setpointGenerator.generateSetpoint(
                _limits,
                _Io.setpoint,
                updatedChassisSpeeds,
                Constants.UPDATE_PERIOD); // updates the setpoint of IO
    }
    public void setModuleStates(SwerveModuleState[] states) {
        for (int module = 0; module < _modules.length; module++) {
            _modules[module].setModuleState(states[module]);
        }
    }
    public void setVelocity(ChassisSpeeds chassisSpeeds) { // this method is main way of interfaceing
        _Io.desiredChassisSpeeds = chassisSpeeds;
    }
    public ChassisSpeeds getMeasuredChassisSpeeds() {
        return _kinematics.toChassisSpeeds(_Io.measuredStates);
    }

    public ChassisSpeeds getDesiredChassisSpeeds() {
        return _Io.desiredChassisSpeeds;
    }
    public void orientModules(Rotation2d moduleAngle) {
        for (int module = 0; module < _modules.length; module++) {
            _Io.setpoint.moduleStates[module] = new SwerveModuleState(0.0, moduleAngle);
        }
    }
    public void setSetpointFromMeasuredModules() {
        System.arraycopy(_Io.measuredStates, 0, _Io.setpoint.moduleStates, 0, _modules.length);
        _Io.setpoint.chassisSpeeds = _kinematics.toChassisSpeeds(_Io.setpoint.moduleStates);
    }
    public SwerveModulePosition[] getSwerveModuleMeasuredPositions() {
        return _Io.measuredPositions;
    }
    public Rotation2d getHeading(){
        return _Io.heading;
    }
    // public double getModulencoder(int mod){
    //     return _modules[mod].getSwervePosition().angle.getDegrees();
    // }
    public void incrementHeadingControllerAngle() {
        Rotation2d heading = getHeading();
        _heading.goToHeading(
                Rotation2d.fromDegrees(heading.getDegrees() + Constants.Drivetrain.BUMP_DEGREES));
    }

    public void decrementHeadingControllerAngle() {
        Rotation2d heading = getHeading();
        _heading.goToHeading(
                Rotation2d.fromDegrees(heading.getDegrees() - Constants.Drivetrain.BUMP_DEGREES));
    }
    public void initializeHeadingController() {
        _heading.goToHeading(getHeading());
    }
    public void setHeadinRotation2D(Rotation2d heading){
        _heading.goToHeading(heading);
    }
    public void readModules() {
        for (int module = 0; module < _modules.length; module++) {
            _Io.measuredPositions[module] = _modules[module].getSwervePosition();
            _Io.measuredStates[module] = _modules[module].getSwerveModuleState();
            // Log encoder values for debugging
            SmartDashboard.putNumber("Module " + module + " Position", _Io.measuredPositions[module].distanceMeters);
            SmartDashboard.putNumber("Module " + module + " Angle", _Io.measuredPositions[module].angle.getDegrees());
            SmartDashboard.putNumber("Module " + module + " State Velocity", _Io.measuredStates[module].speedMetersPerSecond);
            SmartDashboard.putNumber("Module " + module + " State Angle", _Io.measuredStates[module].angle.getDegrees());
        }
    }
    public double getModuleAngle(int module){
        return _Io.measuredPositions[module].angle.getDegrees();
    }
    public void updateShuffleBoard(){
        SmartDashboard.putNumber("Vx", _Io.desiredChassisSpeeds.vxMetersPerSecond);
     SmartDashboard.putNumber("Vy", _Io.desiredChassisSpeeds.vyMetersPerSecond);
      SmartDashboard.putNumber("rotation", _Io.desiredChassisSpeeds.omegaRadiansPerSecond);
       SmartDashboard.putNumber("heading degrees", getHeading().getDegrees());
       SmartDashboard.putNumber("NW_DESIRED_HEADING", getModuleAngle(NORTH_EAST_IDX));
         SmartDashboard.putNumber("NE_DESIRED_HEADING", _Io.measuredPositions[NORTH_EAST_IDX].angle.getDegrees());
           SmartDashboard.putNumber("SW_DESIRED_HEADING", _Io.measuredPositions[SOUTH_WEST_IDX].angle.getDegrees());
             SmartDashboard.putNumber("SE_DESIRED_HEADING", _Io.measuredPositions[SOUTH_EAST_IDX].angle.getDegrees());
             SmartDashboard.putNumber("NW_ACTAUL_HEADING", _modules[NORTH_WEST_IDX].getSwervePosition().angle.getDegrees());
             SmartDashboard.putNumber("NE_ACTAUL_HEADING", _modules[NORTH_EAST_IDX].getSwervePosition().angle.getDegrees());
             SmartDashboard.putNumber("SW_ACTAUL_HEADING", _modules[SOUTH_WEST_IDX].getSwervePosition().angle.getDegrees());
             SmartDashboard.putNumber("SE_ACTAUL_HEADING", _modules[SOUTH_EAST_IDX].getSwervePosition().angle.getDegrees());
    }
    public void ZeroIMU(){
        _yawoffset = _gyro.getYaw();
        readIMU();
    }
    
    public void readIMU() {
        double yawDegrees = _gyro.getYaw();
        double yawAllianceOffsetDegrees = isRedAlliance() ? 180.0 : 0;
        _Io.heading = Rotation2d.fromDegrees(yawDegrees - _yawoffset + yawAllianceOffsetDegrees);
    }
   public boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Red;
        }
        return false;
    }
    

}
