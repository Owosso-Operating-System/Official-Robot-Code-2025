// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Class: IntakeSubsystem
 * Creates a new IntakeSubsystem.
 */

public class IntakeSubsystem extends SubsystemBase {

    public SparkMax intakeT;
    public SparkMax intakeB;

    private SparkMaxConfig intakeTConfig;
    private SparkMaxConfig intakeBConfig;

    /**
     * Method: IntakeSubsysem
     * Parameters: None
     * Variables used: intakeT and intakeB (the top and bottom motors of the intake)
     * What it does: Assigns the SparkMax variables their output ports
     */

    public IntakeSubsystem() {
      
      intakeT = new SparkMax(16, MotorType.kBrushless);
      intakeB = new SparkMax(17, MotorType.kBrushless);

      intakeTConfig = new SparkMaxConfig();
      intakeBConfig = new SparkMaxConfig();

      intakeTConfig.inverted(false).idleMode(IdleMode.kBrake);
      intakeBConfig.inverted(true).idleMode(IdleMode.kBrake);

      intakeT.configure(intakeTConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      intakeB.configure(intakeBConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

}
