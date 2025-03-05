package frc.robot.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Class: LiftSubsystem
 * Creates a new LiftSubsystem.
 */

public class LiftSubsystem extends SubsystemBase {

    public SparkMax liftL;
    public SparkMax liftR;

    public SparkMaxConfig configliftL;
    public SparkMaxConfig configliftR;

    public RelativeEncoder liftPosition;

    /**
     * Method: LiftSubsysem
     * Parameters: None
     * Variables used: LiftL and LiftR (the left and right motors of the lift)
     * What it does: Assigns the SparkMax variables their output ports
     */

    public LiftSubsystem() {

        liftL = new SparkMax(14, MotorType.kBrushless);
        liftR = new SparkMax(15, MotorType.kBrushless);

        configliftL = new SparkMaxConfig();
        configliftR = new SparkMaxConfig();

        configliftL.inverted(false).idleMode(IdleMode.kBrake);
        configliftR.inverted(true).idleMode(IdleMode.kBrake);

        liftL.configure(configliftL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        liftR.configure(configliftR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        liftPosition = liftL.getEncoder();
        liftPosition.setPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //System.out.println("Lift is at :" + liftPosition.getPosition());

    }

}
