package frc.robot.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot.subsystems.IntakeSubsystem;

/**
 * Class: Intake
 * Creates a new Intake Command.
 */
public class Intake extends Command {

    public final IntakeSubsystem intakeSubsystem;
    public final XboxController auxiliaryController;

    /**
     * Method: Intake
     * Parameters: IntakeSubsystem and XboxController
     * Variables used: intakeSubsystem and auxiliaryController
     * What it does: Assigns the parameter IntakeSubsystem to intakeSubsystem
     * Assigns the parameter XboxController to auxiliaryController
     * Uses addRequirements to tie IntakeSubsystem to Intake
     */

    public Intake(IntakeSubsystem intakeSubsystem, XboxController auxiliaryController) {
        this.intakeSubsystem = intakeSubsystem;
        this.auxiliaryController = auxiliaryController;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.intakeSubsystem);
    }

    /**
     * Method: execute
     * Parameters: None
     * Variables used: intakeSubsystem.intakeT, intakeSubsystem.intakeB, and
     * auxiliaryController
     * What it does: Takes the controller outputs, passes the values to intake
     * motors (A for one direction, B for the other)
     */
    @Override
    public void execute() {
        if (auxiliaryController.getAButton()) {
            intakeSubsystem.intakeT.set(-0.25);
            intakeSubsystem.intakeB.set(-0.25);
        } else if (auxiliaryController.getBButton()) {
            intakeSubsystem.intakeT.set(0.25);
            intakeSubsystem.intakeB.set(0.25);
        } else {
            intakeSubsystem.intakeT.set(0);
            intakeSubsystem.intakeB.set(0);
        }
    }
}
