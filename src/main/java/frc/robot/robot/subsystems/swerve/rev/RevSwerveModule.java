
package frc.robot.robot.subsystems.swerve.rev;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
//import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
//import com.revrobotics.SparkMaxLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import frc.robot.lib.util.swerveUtil.CTREModuleState;
import frc.robot.lib.util.swerveUtil.RevSwerveModuleConstants;

/**
 * a Swerve Modules using REV Robotics motor controllers and CTRE CANcoder absolute encoders.
 */
public class RevSwerveModule implements SwerveModule
{
    public int moduleNumber;
    private Rotation2d rotOffset;
    //private Rotation2d lastAngle;

    private SparkMax mAngleMotor;
    private SparkMax mDriveMotor;

    private CANcoder angleEncoder;
    private RelativeEncoder relAngleEncoder;
    private RelativeEncoder relDriveEncoder;
    private SwerveModuleState m_desiredState;

    private boolean lastSpeedNegative = false;

    //SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public RevSwerveModule(int moduleNumber, RevSwerveModuleConstants moduleConstants)
    {
        this.moduleNumber = moduleNumber;
        this.rotOffset = moduleConstants.rotOffset;
        
       
        /* Angle Motor Config */
        mAngleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new SparkMax(moduleConstants.driveMotorID,  MotorType.kBrushless);
        configDriveMotor();

         /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configEncoders();

        //lastAngle = getState().angle;
    }


    private void configEncoders()
    {     
        // absolute encoder  
        CANcoderConfiguration coderConfig = new CANcoderConfiguration();

        coderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        coderConfig.MagnetSensor.MagnetOffset = rotOffset.getRotations();

        angleEncoder.getConfigurator().apply(coderConfig);
        //angleEncoder.configAllSettings(new RevSwerveConfig().canCoderConfig);
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        relDriveEncoder = mDriveMotor.getEncoder();
        relDriveEncoder.setPosition(0);

        motorConfig.encoder.positionConversionFactor(RevSwerveConfig.driveRevToMeters);
        motorConfig.encoder.velocityConversionFactor(RevSwerveConfig.driveRpmToMetersPerSecond);

        relAngleEncoder = mAngleMotor.getEncoder();
        relAngleEncoder.setPosition(0);
        // in degrees/sec
        motorConfig.encoder.velocityConversionFactor(RevSwerveConfig.DegreesPerTurnRotation / 60);
        
        resetToAbsolute();
        //mDriveMotor.burnFlash();
        //mAngleMotor.burnFlash();    
    }

    private void configAngleMotor()
    {        
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        ClosedLoopConfig controller = new ClosedLoopConfig();
        
        mAngleMotor.configure(motorConfig, ResetMode.kResetSafeParameters, null);
        controller.pidf(RevSwerveConfig.angleKP, RevSwerveConfig.angleKI, RevSwerveConfig.angleKD,RevSwerveConfig.angleKF);
        controller.outputRange(-RevSwerveConfig.anglePower, RevSwerveConfig.anglePower);
    
        motorConfig.inverted(RevSwerveConfig.angleMotorInvert);
        motorConfig.smartCurrentLimit(RevSwerveConfig.angleContinuousCurrentLimit);
        motorConfig.idleMode(RevSwerveConfig.angleIdleMode);
        motorConfig.apply(controller);


        mAngleMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
    }

    private void configDriveMotor()
    {        
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        ClosedLoopConfig controller = new ClosedLoopConfig();
        
        mDriveMotor.configure(motorConfig, ResetMode.kResetSafeParameters, null);
        controller.pidf(RevSwerveConfig.driveKP, RevSwerveConfig.driveKI, RevSwerveConfig.driveKD,RevSwerveConfig.driveKF);
        controller.outputRange(-RevSwerveConfig.drivePower, RevSwerveConfig.drivePower);
       
        if(moduleNumber == 1 || moduleNumber == 3){
            motorConfig.inverted(true);
        }
        else{
            motorConfig.inverted(false);
        }
        
        motorConfig.smartCurrentLimit(RevSwerveConfig.driveContinuousCurrentLimit);
        motorConfig.idleMode(RevSwerveConfig.driveIdleMode);
        motorConfig.apply(controller);

        mDriveMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private SwerveModuleState optimizeAngle(SwerveModuleState desiredState, Rotation2d rotation2d)
    {
        double roundScale = 1000;

        double desiredAngle = Math.round(getRotationBound(desiredState.angle.getRotations()) * roundScale)/roundScale;
        double desiredAngleI;
        
        if(desiredAngle > 0){
            desiredAngleI = Math.round(getRotationBound(desiredAngle - 0.5) * roundScale)/roundScale;
        }else{
            desiredAngleI = Math.round(getRotationBound(desiredAngle + 0.5) * roundScale)/roundScale;
        }

        double currentAngle;

        if(!lastSpeedNegative){
            currentAngle = Math.round(getRotationBound(rotation2d.getRotations() / RevSwerveConfig.angleGearRatio) * roundScale)/roundScale;
        }else{
            currentAngle = -Math.round(getRotationBound(rotation2d.getRotations() / RevSwerveConfig.angleGearRatio) * roundScale)/roundScale;
        }

        double deltaAngleR =  Math.round(getRotationBound(desiredAngle - currentAngle) * roundScale)/roundScale;
        double deltaAngleI =  Math.round(getRotationBound(desiredAngleI - currentAngle) * roundScale)/roundScale;

        //
        if((Math.abs(deltaAngleI) > Math.abs(deltaAngleR))){
            lastSpeedNegative = false;
            desiredState.angle = Rotation2d.fromRotations((desiredAngle));
        }else if(Math.abs(deltaAngleI) < Math.abs(deltaAngleR)){
            lastSpeedNegative = true;
            desiredState.speedMetersPerSecond = -desiredState.speedMetersPerSecond;
            desiredState.angle = Rotation2d.fromRotations((desiredAngleI));
        }else if(Math.abs(deltaAngleI) == Math.abs(deltaAngleR)){
            if(!lastSpeedNegative){
                lastSpeedNegative = true;
                desiredState.angle = Rotation2d.fromRotations((desiredAngle));
            }else{
                lastSpeedNegative = false;
                desiredState.speedMetersPerSecond = -desiredState.speedMetersPerSecond;
                desiredState.angle = Rotation2d.fromRotations((desiredAngleI));
            }
        }

        return desiredState;
    }

    public double getRotationBound(double angle){
        if(angle > 1){
            angle = angle - 1;
            return getRotationBound(angle);
        }else if(angle < -1){
            angle = angle + 1;
            return getRotationBound(angle);
        }else{
            return angle;
        }
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        // CTREModuleState actually works for any type of motor.
        // desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        
        //System.out.println("Angle CANCODER Encoder: " + moduleNumber + '\n' + ' ' + angleEncoder.getPosition().getValueAsDouble());

        desiredState = optimizeAngle(desiredState, getState().angle);
        
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop)
    {
       
        if(isOpenLoop)
        {
            double percentOutput = desiredState.speedMetersPerSecond * RevSwerveConfig.maxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }
 
        double velocity = desiredState.speedMetersPerSecond;
        
        SparkClosedLoopController controller = mDriveMotor.getClosedLoopController();
        controller.setReference(velocity, ControlType.kVelocity);

        if(Math.abs(desiredState.speedMetersPerSecond) <= (RevSwerveConfig.maxSpeed * 0.01)) 
        {
            mDriveMotor.stopMotor();
            return;
        }
    }

    private void setAngle(SwerveModuleState desiredState)
    {
        /*
        if(Math.abs(desiredState.speedMetersPerSecond) <= (RevSwerveConfig.maxSpeed * 0.01)) 
        {
            mAngleMotor.stopMotor();
            return;
        }
        */
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.       
        SparkClosedLoopController controller = mAngleMotor.getClosedLoopController();
        
        controller.setReference(desiredState.angle.getRotations() * RevSwerveConfig.angleGearRatio, ControlType.kPosition);
        
    }

    public SwerveModuleState getDesiredState() {
        return m_desiredState;
    }

    public Rotation2d getAngle()
    {
        return Rotation2d.fromRotations(angleEncoder.getPosition().getValueAsDouble());
    }

    public Rotation2d getDesiredAngle() {
        return getDesiredState().angle;
    }

    public double getDesiredVelocity() {
        return getDesiredState().speedMetersPerSecond;
    }

    public Rotation2d getCANcoder()
    {
        return getAngle();
    }

    public int getModuleNumber() 
    {
        return moduleNumber;
    }

    public void setModuleNumber(int moduleNumber) 
    {
        this.moduleNumber = moduleNumber;
    }

    public void resetToAbsolute()
    {
        relAngleEncoder.setPosition(-(angleEncoder.getAbsolutePosition().getValueAsDouble()) * RevSwerveConfig.angleGearRatio);
        relDriveEncoder.setPosition(-(angleEncoder.getAbsolutePosition().getValueAsDouble()) * RevSwerveConfig.angleGearRatio);
    }
  

    public SwerveModuleState getState()
    {
        return new SwerveModuleState(
            relDriveEncoder.getVelocity(),
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(
            relDriveEncoder.getPosition(), 
            getAngle()
        );
    }

    public void stopMotor(){
        SwerveModuleState stopModuleState = new SwerveModuleState();
        stopModuleState.speedMetersPerSecond = 0;

        setAngle(stopModuleState);
        setSpeed(stopModuleState, true);
    }

}