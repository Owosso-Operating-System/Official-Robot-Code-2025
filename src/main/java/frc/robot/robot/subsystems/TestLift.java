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

public class TestLift extends SubsystemBase {
  public SparkMax liftL;
  private SparkMaxConfig liftLConfig;
  public SparkMax liftR;
  private SparkMaxConfig liftRConfig;

  /** Creates a new TestLift. */
  public TestLift() {
      liftL = new SparkMax(14, MotorType.kBrushless);
      liftR = new SparkMax(15, MotorType.kBrushless);

      liftLConfig = new SparkMaxConfig();
      liftRConfig = new SparkMaxConfig();

      liftRConfig.inverted(true);

      liftL.configure(liftLConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      liftR.configure(liftRConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



}
