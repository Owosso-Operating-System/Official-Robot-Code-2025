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
    public final XboxController controller1;

    /**
     * Method: Lift
     * Parameters: LiftSubsystem and XboxController
     * Variables used: liftSubsystem and controller1
     * What it does: Assigns the parameter LiftSubsystem to liftSubsystem
     * Assigns the parameter XboxController to controller1
     * Uses addRequirements to tie LiftSubsystem to Lift
     */

    public Lift(LiftSubsystem liftSubsystem, XboxController controller1) {
        this.liftSubsystem = liftSubsystem;
        this.controller1 = controller1;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.liftSubsystem);
    }

    /**
     * Method: execute
     * Parameters: None
     * Variables used: liftSubsystem.liftL, liftSubsystem.liftR, and controller1
     * What it does: Takes the controller outputs, passes the values to lift motors
     * (LT for one direction, RT for the other)
     */
    @Override
    public void execute() {
        if (controller1.getRawAxis(2) > .2) {
            liftSubsystem.liftL.set(controller1.getRawAxis(2));
            liftSubsystem.liftR.set(controller1.getRawAxis(2));
        } else if (controller1.getRawAxis(3) > .2) {
            liftSubsystem.liftL.set(-controller1.getRawAxis(3));
            liftSubsystem.liftR.set(-controller1.getRawAxis(3));
        } else {
            liftSubsystem.liftL.set(0);
            liftSubsystem.liftR.set(0);
        }
    }
}
