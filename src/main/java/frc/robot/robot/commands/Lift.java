package frc.robot.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot.subsystems.LiftSubsystem;

public class Lift extends Command {
    public final LiftSubsystem liftSubsystem;
    public final XboxController controller1;
    
    public Lift(LiftSubsystem liftSubsystem, XboxController controller1){
        this.liftSubsystem = liftSubsystem;
        this.controller1 = controller1;

        addRequirements(this.liftSubsystem);
    }

    @Override
    public void execute(){
        if(controller1.getRawAxis(2) > .2){
            liftSubsystem.liftL.set(controller1.getRawAxis(2));
            liftSubsystem.liftR.set(controller1.getRawAxis(2));
        } else if(controller1.getRawAxis(3) > .2){
            liftSubsystem.liftL.set(controller1.getRawAxis(3));
            liftSubsystem.liftR.set(controller1.getRawAxis(3));
        } else{
            liftSubsystem.liftL.set(0);
            liftSubsystem.liftR.set(0);
        }
    }
}
