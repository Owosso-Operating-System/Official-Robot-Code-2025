// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public SparkMax intakeT;
  private SparkMaxConfig intakeTConfig;
  public SparkMax intakeB;
  private SparkMaxConfig intakeBConfig;

  /** Creates a new intake. */
  public Intake() {
      intakeT = new SparkMax(16, MotorType.kBrushless);
      intakeB = new SparkMax(17, MotorType.kBrushless);
  //configutrs the imtake motors
      intakeTConfig = new SparkMaxConfig();
      intakeBConfig = new SparkMaxConfig();
  // inverts bottom motor
      intakeBConfig.inverted(true);
  
      intakeT.configure(intakeTConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      intakeB.configure(intakeBConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



}
