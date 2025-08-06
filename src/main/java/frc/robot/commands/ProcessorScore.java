package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;

public class ProcessorScore extends Command {

    Intake INTAKE;

    public ProcessorScore(Intake intake){
        this.INTAKE = intake;
        addRequirements(intake);
    }

    public void initialize()
    {
       INTAKE.toSetpoint(1);
       INTAKE.setRollers(60); 
    }
    
    public void execute()
    {
        isFinished();
    }

    @Override
    public boolean isFinished() {
        return true;
    }  
}