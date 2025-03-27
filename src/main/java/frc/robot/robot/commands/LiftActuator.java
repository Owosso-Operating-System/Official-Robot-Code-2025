package frc.robot.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot.subsystems.LiftActuatorSubsystem;

/**
 * Class: LiftActuator
 * Creates a new LiftActuator Command.
 */
public class LiftActuator extends Command {

    public final LiftActuatorSubsystem liftActuatorSubsystem;
    public final XboxController driveController;

    /**
     * Method: LiftActuator
     * Parameters: liftActuatorSubsystem and driverController
     * Variables used: liftActuatorSubsystem and driverController
     * What it does: Assigns the parameter LiftActuatorSubsystem to liftActuatorSubsystem
     * Assigns the parameter XboxController to driveController
     * Uses addRequirements to tie LiftActuatorSubsystem to LiftActuator
     */

    public LiftActuator(LiftActuatorSubsystem liftActuatorSubsystem, XboxController driveController) {
        this.liftActuatorSubsystem = liftActuatorSubsystem;
        this.driveController = driveController;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.liftActuatorSubsystem);
    }

    /**
     * Method: execute
     * Parameters: None
     * Variables used: liftActuatorSubsystem.ActuatorL, liftActuatorSubsystem.ActuatorR, and driveController
     * What it does: Takes the controller outputs, passes the values to lift actuator motors
     * (LT for one direction, RT for the other)
     */
    @Override
    public void execute() {
        if (driveController.getRawAxis(2) > 0.1) {
            liftActuatorSubsystem.ActuatorL.set(-driveController.getRawAxis(2) * 0.5);
            liftActuatorSubsystem.ActuatorR.set(-driveController.getRawAxis(2) * 0.5);
        } else if (driveController.getRawAxis(3) > 0.1) {
            liftActuatorSubsystem.ActuatorL.set(driveController.getRawAxis(3) * 0.5);
            liftActuatorSubsystem.ActuatorR.set(driveController.getRawAxis(3) * 0.5);
        }else {
            liftActuatorSubsystem.ActuatorL.set(0);
            liftActuatorSubsystem.ActuatorR.set(0);
        }
    }
}
