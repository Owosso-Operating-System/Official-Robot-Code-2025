// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot.subsystems.LiftSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LiftCoralOne extends Command {

    public final LiftSubsystem liftSubsystem;
    public final XboxController auxiliaryController;

    /**
     * Method: LiftCoralOne
     * Parameters: LiftSubsystem and XboxController
     * Variables used: liftSubsystem and auxiliaryController
     * What it does: Assigns the parameter LiftSubsystem to liftSubsystem
     * Assigns the parameter XboxController to auxiliaryController
     * Uses addRequirements to tie LiftSubsystem to LiftCoralOne
     */

    public LiftCoralOne(LiftSubsystem liftSubsystem, XboxController auxiliaryController) {
        this.liftSubsystem = liftSubsystem;
        this.auxiliaryController = auxiliaryController;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.liftSubsystem);
    }
    // Use addRequirements() here to declare subsystem dependencies.

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    final double targetLB = 25.35697;

    if(auxiliaryController.getLeftBumperButton() && (auxiliaryController.getRawAxis(1) < 0.1 && auxiliaryController.getRawAxis(1) > -0.1)){
        if(liftSubsystem.liftPosition.getPosition() < targetLB - 0.5){
            liftSubsystem.liftL.set(0.3);
            liftSubsystem.liftR.set(0.3);
        } else if(liftSubsystem.liftPosition.getPosition() > targetLB + 0.5){
            liftSubsystem.liftL.set(-0.3);
            liftSubsystem.liftR.set(-0.3);
        }else{
            liftSubsystem.liftL.set(0);
            liftSubsystem.liftR.set(0);   
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
