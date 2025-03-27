package frc.robot.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;

public class LiftActuatorSubsystem extends SubsystemBase{

    public SparkMax ActuatorL;
    public SparkMax ActuatorR;

    public SparkMaxConfig configActuatorL;
    public SparkMaxConfig configActuatorR;

    public RelativeEncoder ActuatorPosition;


    public LiftActuatorSubsystem() {

        ActuatorL = new SparkMax(18, MotorType.kBrushless);
        ActuatorR = new SparkMax(19, MotorType.kBrushless);

        configActuatorL = new SparkMaxConfig();
        configActuatorR = new SparkMaxConfig();

        //configActuatorL.inverted(false).idleMode(IdleMode.kBrake);
        //configActuatorR.inverted(true).idleMode(IdleMode.kBrake);

        ActuatorL.configure(configActuatorL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        ActuatorR.configure(configActuatorR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        ActuatorPosition = ActuatorL.getEncoder();
        ActuatorPosition.setPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

}


