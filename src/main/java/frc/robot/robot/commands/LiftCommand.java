// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot.subsystems.TestLift;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LiftCommand extends Command {
  public final TestLift testLift;
  public final XboxController controller1;

  /** Creates a new LiftCommand. */
  public LiftCommand(TestLift testLift, XboxController controller1) {
    this.testLift = testLift;
    this.controller1 = controller1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(testLift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(controller1.getRawAxis(4) > 0.1){
      testLift.liftL.set(controller1.getRawAxis(4) * 0.5);
    }else if(controller1.getRawAxis(4) < -0.1){
      testLift.liftR.set(controller1.getRawAxis(4) * -0.5);
    }else{
      testLift.liftL.set(0);
      testLift.liftR.set(0);
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
