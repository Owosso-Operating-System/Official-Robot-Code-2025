package frc.robot.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot.subsystems.LiftSubsystem;

/**
 * Class: Lift
 * Creates a new Lift Command.
 */
public class Lift extends Command {

    public final LiftSubsystem liftSubsystem;
    public final XboxController auxiliaryController;

    /**
     * Method: Lift
     * Parameters: LiftSubsystem and XboxController
     * Variables used: liftSubsystem and auxiliaryController
     * What it does: Assigns the parameter LiftSubsystem to liftSubsystem
     * Assigns the parameter XboxController to auxiliaryController
     * Uses addRequirements to tie LiftSubsystem to Lift
     */

    public Lift(LiftSubsystem liftSubsystem, XboxController auxiliaryController) {
        this.liftSubsystem = liftSubsystem;
        this.auxiliaryController = auxiliaryController;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.liftSubsystem);
    }

    /**
     * Method: execute
     * Parameters: None
     * Variables used: liftSubsystem.liftL, liftSubsystem.liftR, and auxiliaryController
     * What it does: Takes the controller outputs, passes the values to lift motors
     * (LT for one direction, RT for the other)
     */
    @Override
    public void execute() {
        if (auxiliaryController.getRawAxis(1) > 0.1) {
            liftSubsystem.liftL.set(-auxiliaryController.getRawAxis(1) * 0.5);
            liftSubsystem.liftR.set(-auxiliaryController.getRawAxis(1) * 0.5);
        } else if (auxiliaryController.getRawAxis(1) < -0.1) {
            liftSubsystem.liftL.set(-auxiliaryController.getRawAxis(1) * 0.5);
            liftSubsystem.liftR.set(-auxiliaryController.getRawAxis(1) * 0.5);
        }else {
            liftSubsystem.liftL.set(0);
            liftSubsystem.liftR.set(0);
        }
    }
}
