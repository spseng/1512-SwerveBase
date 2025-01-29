package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Utils.SwerveHeadingController;
import frc.robot.Utils.SwerveSetpoint;
import frc.robot.Utils.SwerveSetpointGenerator;
import frc.robot.Utils.SwerveSetpointGenerator.KinematicLimits;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Drivetrain extends SubsystemBase {

    private static final int NORTH_EAST_IDX = 0;
    private static final int NORTH_WEST_IDX = 1;
    private static final int SOUTH_EAST_IDX = 2;
    private static final int SOUTH_WEST_IDX = 3;
    private final SwerveSetpointGenerator _setpointGenerator;
    private KinematicLimits _limits;
    private final SwerveDriveKinematics _kinematics; // physical layout of chassis
    //private final AHRS _gyro; // navX might will be changed to pigeon 2.0 soon
    private final Pigeon2 _pigeon;
    private final SwerveModule[] _modules;
    private double _yawOffset;

    private final SwerveHeadingController _heading;

    private final SystemIO _Io;

    StructArrayPublisher<SwerveModuleState> desiredSwerveStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("DesiredSwerveStates", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> measuredSwerveStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("MeasuredSwerveStates", SwerveModuleState.struct).publish();
    StructArrayPublisher<Pose2d> currentPosePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("CurrentPose", Pose2d.struct).publish();


    private final SwerveDriveOdometry _odometry;

    private Pose2d _current_pose;
    private Pose2d _previous_pose;

    private final PoseSupplier _currentPoseSupplier = new PoseSupplier();
    private final ChassisSpeedsSupplier _currentChassisSpeedsSupplier = new ChassisSpeedsSupplier();
    PathFollowingController _controller;

    public Drivetrain() {
        _Io = new SystemIO();
        //_gyro = new AHRS(SPI.Port.kMXP); // I think that this is right
        _pigeon = new Pigeon2(RobotMap.CAN.PIGEON_CAN);

        //_yawOffset = _gyro.getYaw(); // zero gyro on init
        _yawOffset = _pigeon.getYaw().getValueAsDouble();
        readIMU(); // method to update gyro

        _modules = new SwerveModule[4];

        _modules[NORTH_WEST_IDX] = new SwerveModule(RobotMap.CAN.FL_STEER_CAN, RobotMap.CAN.FL_DRIVE_CAN, Constants.Drivetrain.NORTH_WEST_CONFIG); // TODO CHANGES
        _modules[NORTH_EAST_IDX] = new SwerveModule(RobotMap.CAN.FR_STEER_CAN, RobotMap.CAN.FR_DRIVE_CAN, Constants.Drivetrain.NORTH_EAST_CONFIG); // TODO CHANGES
        _modules[SOUTH_WEST_IDX] = new SwerveModule(RobotMap.CAN.BL_STEER_CAN, RobotMap.CAN.BL_DRIVE_CAN, Constants.Drivetrain.SOUTH_WEST_CONFIG); // TODO CHANGES
        _modules[SOUTH_EAST_IDX] = new SwerveModule(RobotMap.CAN.BR_STEER_CAN, RobotMap.CAN.BR_DRIVE_CAN, Constants.Drivetrain.SOUTH_EAST_CONFIG); // TODO CHANGES


        _kinematics = new SwerveDriveKinematics( //location in where it is on chassis
                _modules[NORTH_EAST_IDX].getSwerveModuleLocation(),
                _modules[NORTH_WEST_IDX].getSwerveModuleLocation(),
                _modules[SOUTH_EAST_IDX].getSwerveModuleLocation(),
                _modules[SOUTH_WEST_IDX].getSwerveModuleLocation());

        _odometry = new SwerveDriveOdometry(_kinematics, getHeading(), new SwerveModulePosition[]{
                _modules[NORTH_WEST_IDX].getSwervePosition(),
                _modules[NORTH_EAST_IDX].getSwervePosition(),
                _modules[SOUTH_WEST_IDX].getSwervePosition(),
                _modules[SOUTH_EAST_IDX].getSwervePosition()
        }, new Pose2d());

        _current_pose = _odometry.getPoseMeters();
        _previous_pose = _current_pose;

        _setpointGenerator = new SwerveSetpointGenerator(
                _kinematics,
                new Translation2d[]{
                        _modules[NORTH_EAST_IDX].getSwerveModuleLocation(),
                        _modules[NORTH_WEST_IDX].getSwerveModuleLocation(),
                        _modules[SOUTH_EAST_IDX].getSwerveModuleLocation(),
                        _modules[SOUTH_WEST_IDX].getSwerveModuleLocation()

                }); // cheesy stuff
        _heading = new SwerveHeadingController(0.2);     // not sure if we want to use this
        _limits = Constants.Drivetrain.DRIVE_KINEMATIC_LIMITS;
        ZeroIMU(); // resets heading
        readModules(); // gets encoders
        setSetpointFromMeasuredModules(); // cheesy stuff

        PathFollowingController _controller = new PathFollowingController() {
            @Override
            public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose, PathPlannerTrajectoryState targetState) {
                return null;
            }

            @Override
            public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {

            }

            @Override
            public boolean isHolonomic() {
                return true;
            }
        };
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
        updateSwerveOdometry();
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
        this.desiredSwerveStatePublisher.set(states);
    }

    public void setVelocity(ChassisSpeeds chassisSpeeds) { // this method is the main way of interfacing
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

    public Rotation2d getHeading() {
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

    public void setKinematicLimits(KinematicLimits limits) {
        if (limits != _limits) {
            _limits = limits;
        }
    }


    public void decrementHeadingControllerAngle() {
        Rotation2d heading = getHeading();
        _heading.goToHeading(
                Rotation2d.fromDegrees(heading.getDegrees() - Constants.Drivetrain.BUMP_DEGREES));
    }

    public void initializeHeadingController() {
        _heading.goToHeading(getHeading());
    }

    public void setHeadingRotation2D(Rotation2d heading) {
        _heading.goToHeading(heading);
    }

    public void readModules() {
        for (int module = 0; module < _modules.length; module++) {
            _Io.measuredPositions[module] = _modules[module].getSwervePosition();
            _Io.measuredStates[module] = _modules[module].getSwerveModuleState();

            // Multiply the angle by -1 for the SwerveModuleState
            double adjustedAngleRadians = (_Io.measuredStates[module].angle.getRadians());
            _Io.measuredStates[module].angle = new Rotation2d(adjustedAngleRadians);
            // Log encoder values for debugging
            SmartDashboard.putNumber("Module " + module + " Position", _Io.measuredPositions[module].distanceMeters);
            SmartDashboard.putNumber("Module " + module + " Angle", _Io.measuredPositions[module].angle.getDegrees());
            SmartDashboard.putNumber("Module " + module + " State Velocity", _Io.measuredStates[module].speedMetersPerSecond);
            SmartDashboard.putNumber("Module " + module + " State Angle", _Io.measuredStates[module].angle.getDegrees());
        }
        this.measuredSwerveStatePublisher.set(_Io.measuredStates);
    }

    public double getModuleAngle(int module) {
        return _Io.measuredPositions[module].angle.getDegrees();
    }

    public void updateShuffleBoard() {
        SmartDashboard.putNumber("Vx", _Io.desiredChassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Vy", _Io.desiredChassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("desired rotation", _Io.desiredChassisSpeeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("heading degrees", getHeading().getDegrees());
        SmartDashboard.putNumber("heading radians", -getHeading().getRadians());
        SmartDashboard.putNumber("NW_DESIRED_HEADING", _Io.measuredPositions[NORTH_WEST_IDX].angle.getDegrees());
        SmartDashboard.putNumber("NE_DESIRED_HEADING", _Io.measuredPositions[NORTH_EAST_IDX].angle.getDegrees());
        SmartDashboard.putNumber("SW_DESIRED_HEADING", _Io.measuredPositions[SOUTH_WEST_IDX].angle.getDegrees());
        SmartDashboard.putNumber("SE_DESIRED_HEADING", _Io.measuredPositions[SOUTH_EAST_IDX].angle.getDegrees());
        SmartDashboard.putNumber("NW_ACTUAL_HEADING", _modules[NORTH_WEST_IDX].getSwervePosition().angle.getDegrees());
        SmartDashboard.putNumber("NE_ACTUAL_HEADING", _modules[NORTH_EAST_IDX].getSwervePosition().angle.getDegrees());
        SmartDashboard.putNumber("SW_ACTUAL_HEADING", _modules[SOUTH_WEST_IDX].getSwervePosition().angle.getDegrees());
        SmartDashboard.putNumber("SE_ACTUAL_HEADING", _modules[SOUTH_EAST_IDX].getSwervePosition().angle.getDegrees());
        SmartDashboard.putNumber("NW_ACTUAL_VELOCITY", _modules[NORTH_WEST_IDX].getDriveVelocity());
        SmartDashboard.putNumber("NE_ACTUAL_VELOCITY", _modules[NORTH_EAST_IDX].getDriveVelocity());
        SmartDashboard.putNumber("SW_ACTUAL_VELOCITY", _modules[SOUTH_WEST_IDX].getDriveVelocity());
        SmartDashboard.putNumber("SE_ACTUAL_VELOCITY", _modules[SOUTH_EAST_IDX].getDriveVelocity());
        SmartDashboard.putNumber("ACTUAL_VELOCITY_X", getMeasuredVelocityX());
        SmartDashboard.putNumber("ACTUAL_VELOCITY_Y", getMeasuredVelocityY());
        SmartDashboard.putNumber("ACTUAL_VELOCITY", getMeasuredVelocity());
        //SmartDashboard.putNumber("IMU_VELOCITY", getIMUVelocity());
        //SmartDashboard.putNumber("AVERAGE_VELOCITY", getAverageVelocity());

    }

    public void ZeroIMU() {
        //_yawOffset = _gyro.getYaw();
        _yawOffset = _pigeon.getYaw().getValueAsDouble();
        readIMU();
    }

    public void readIMU() {
        //double yawDegrees = _gyro.getYaw();
        double yawDegrees = _pigeon.getYaw().getValueAsDouble();
        double yawAllianceOffsetDegrees = isRedAlliance() ? 180.0 : 0;
        _Io.heading = Rotation2d.fromDegrees(yawDegrees - _yawOffset + yawAllianceOffsetDegrees);
    }

    public boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == Alliance.Red).isPresent();
    }

    public static class SystemIO { // this is a wrapper for all inputs and output to drive
        ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0); // chassis speeds is a velocity in x & y and rotation in radians per seconds
        SwerveModuleState[] measuredStates = new SwerveModuleState[]{ // state array to control modules
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };
        SwerveModulePosition[] measuredPositions = new SwerveModulePosition[]{ // position is robot relative
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };
        Rotation2d heading = new Rotation2d(0.0);

        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[4]); // cheesy stuff
        //Desired Chassis speeds
        //heading
        //setpoint
        //positions
        //states
    }

    public static class PoseSupplier implements Supplier<Pose2d> {
        private Pose2d currentPose;

        @Override
        public Pose2d get() {
            return currentPose;
        }

        public void updatePose(Pose2d newPose) {
            this.currentPose = newPose;
        }
    }

    public static class ChassisSpeedsSupplier implements Supplier<ChassisSpeeds> {
        private ChassisSpeeds currentChassisSpeeds;

        @Override
        public ChassisSpeeds get() {
            return currentChassisSpeeds;
        }

        public void updateChassisSpeeds(ChassisSpeeds newChassisSpeeds) {
            this.currentChassisSpeeds = newChassisSpeeds;
        }
    }

    public void updateSwerveOdometry() {
        _previous_pose = _current_pose;
        _current_pose = _odometry.update(getHeading(), new SwerveModulePosition[]{
                _modules[NORTH_WEST_IDX].getSwervePosition(),
                _modules[NORTH_EAST_IDX].getSwervePosition(),
                _modules[SOUTH_WEST_IDX].getSwervePosition(),
                _modules[SOUTH_EAST_IDX].getSwervePosition()
        });
        _currentPoseSupplier.updatePose(_current_pose);
        _currentChassisSpeedsSupplier.updateChassisSpeeds(getMeasuredChassisSpeeds());
        currentPosePublisher.set(new Pose2d[]{_current_pose});
    }

    public double getMeasuredVelocityX() {
        return (_current_pose.getTranslation().getX() - _previous_pose.getTranslation().getX()) / Constants.UPDATE_PERIOD;
    }

    public double getMeasuredVelocityY() {
        return (_current_pose.getTranslation().getY() - _previous_pose.getTranslation().getY()) / Constants.UPDATE_PERIOD;
    }

    public double getMeasuredVelocity() {
        return Math.sqrt(Math.pow(getMeasuredVelocityX(), 2) + Math.pow(getMeasuredVelocityY(), 2));
    }

    /*
    public double getIMUVelocity() {
        return Math.sqrt(Math.pow(_gyro.getVelocityX(), 2) + Math.pow(_gyro.getVelocityY(), 2));
    }
    */

    /*
    public double getAverageVelocity() {
        return (getIMUVelocity() + getMeasuredVelocity()) / 2;
    }
    */

    public Pose2d getCurrentPose() {
        return _current_pose;
    }

    public Supplier<Pose2d> currentPoseSupplier() {
        return () -> _current_pose;
    }

    public Supplier<ChassisSpeeds> currentChassisSpeedsSupplier() {
        return () -> _kinematics.toChassisSpeeds(_Io.measuredStates);
    }

    public Consumer<Pose2d> resetCurrentPoseConsumer() {
        return pose -> {
            _current_pose = pose;  // Assuming you want to reset _current_pose to the passed pose
        };
    }

    public BooleanSupplier isRedAllianceSupplier() {
        return () -> {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            return alliance.filter(value -> value == Alliance.Red).isPresent();
        };
    }

    public PathFollowingController getController() {
        return _controller;
    }
}