package frc.robot.robot.subsystems.swerve.rev;

import frc.robot.lib.math.GeometryUtils;
import frc.robot.robot.constants.RevSwerveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
//import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RevSwerve extends SubsystemBase {


    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public RevSwerveModule[] mRevSwerveMods;
    public Pigeon2 gyro;

    

    public RevSwerve() {
        
        gyro = new Pigeon2(RevSwerveConstants.REV.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        
     

        mSwerveMods = new SwerveModule[] {
           
            new RevSwerveModule(0, RevSwerveConstants.Swerve.Mod0.constants),
            new RevSwerveModule(1, RevSwerveConstants.Swerve.Mod1.constants),
            new RevSwerveModule(2, RevSwerveConstants.Swerve.Mod2.constants),
            new RevSwerveModule(3, RevSwerveConstants.Swerve.Mod3.constants)
        };

        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(RevSwerveConfig.swerveKinematics, getYaw(), getModulePositions());
        zeroGyro();

        RobotConfig config = null;
            try{
            config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            }

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    config, // The robot configuration
                    () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                    },
                    this // Reference to this subsystem to set requirements
            );
    }
    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds desiredChassisSpeeds =
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
        translation.getX(),
        translation.getY(),
        rotation,
        getYaw())
        : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation);
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);

        SwerveModuleState[] swerveModuleStates = RevSwerveConfig.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, RevSwerveConfig.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }

    }    
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {

       // System.out.println("setting module states: "+desiredStates[0]);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, RevSwerveConfig.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], true);
        }
    }    
    public Pose2d getPose() {
        Pose2d p =  swerveOdometry.getPoseMeters();
        return new Pose2d(-p.getX(), -p.getY(),  p.getRotation());
    }
    public void resetOdometry(Pose2d pose) {
        
        swerveOdometry.resetPosition(new Rotation2d(), getModulePositions(), pose);
        zeroGyro(pose.getRotation().getDegrees());
       
    }
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void driveRobotRelative(ChassisSpeeds speeds){
        Translation2d targetTranslation2d = new Translation2d(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond);
        this.drive(targetTranslation2d, speeds.omegaRadiansPerSecond,false,false);
      }
      
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return RevSwerveConfig.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void zeroGyro(double deg) {
        if(RevSwerveConfig.invertGyro) {
            deg = -deg;
        }
        gyro.setYaw(deg);
        swerveOdometry.update(getYaw(), getModulePositions());  
    }

    public void zeroGyro() {  
       zeroGyro(0);
    }

    public Rotation2d getYaw() {
        return (RevSwerveConfig.invertGyro) ? Rotation2d.fromDegrees((gyro.getYaw().getValueAsDouble())) : Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void stopMotor(){
        for(SwerveModule mod : mSwerveMods){
            mod.stopMotor();
        }
    }

    @Override
    public void periodic() {
        for(SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Cancoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        swerveOdometry.update(getYaw(), getModulePositions());

    }
}