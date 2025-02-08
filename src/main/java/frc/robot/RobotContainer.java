package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.robot.commands.*;
import frc.robot.robot.subsystems.IntakeSubsystem;
import frc.robot.robot.subsystems.LiftSubsystem;
import frc.robot.robot.subsystems.swerve.rev.RevSwerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driveController = new XboxController(0);
    private final XboxController auxiliaryController = new XboxController(1);   

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driveController, XboxController.Button.kY.value);

    /* Auxiliary Buttons */
    private final JoystickButton intakeIn = new JoystickButton(auxiliaryController, XboxController.Button.kA.value);
    private final JoystickButton intakeOut = new JoystickButton(auxiliaryController, XboxController.Button.kB.value);

    /* Subsystems */
    private final RevSwerve s_Swerve = new RevSwerve();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();    
    private final LiftSubsystem liftSubsystem = new LiftSubsystem();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        liftSubsystem.setDefaultCommand(new Lift(liftSubsystem, auxiliaryController));

        Timer.delay(1.0);
        s_Swerve.resetModulesToAbsolute();

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driveController.getRawAxis(translationAxis), 
                () -> -driveController.getRawAxis(strafeAxis), 
                () -> -driveController.getRawAxis(rotationAxis), 
                () -> false
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        /*Auxiliary Buttons */
        intakeIn.onTrue(new Intake(intakeSubsystem, auxiliaryController));
        intakeOut.onTrue(new Intake(intakeSubsystem, auxiliaryController));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Timer.delay(1.0);
        s_Swerve.resetModulesToAbsolute();

        return null;
    }
}
