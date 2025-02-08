package frc.robot.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Class: LiftSubsystem
   * Creates a new LiftSubsystem.
   *  */

public class LiftSubsystem extends SubsystemBase{
    
    public SparkMax liftL;
    public SparkMax liftR;

    /**Method: LiftSubsysem
    * Parameters: None
    * Variables used: LiftL and LiftR (the left and right motors of the lift)
    * What it does: Assigns the SparkMax variables their output ports
    *  */

    public LiftSubsystem(){

        // initialize the first Spark Motor
        liftL = new SparkMax(14, MotorType.kBrushless);
        // configure the first Spark Motor
        SparkMaxConfig configliftL = new SparkMaxConfig();

        configliftL
            .inverted(false)
            .idleMode(IdleMode.kBrake);
    
        liftL.configure(configliftL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // as above but with the second (inverted)
        liftR = new SparkMax(15, MotorType.kBrushless);
        SparkMaxConfig configliftR = new SparkMaxConfig();

        configliftR
            .inverted(true)
            .idleMode(IdleMode.kBrake);
    
        liftR.configure(configliftR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
