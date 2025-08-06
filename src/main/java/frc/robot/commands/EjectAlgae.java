package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;

public class EjectAlgae extends Command{
    Claw CLAW;
    Intake INTAKE;

    public EjectAlgae(Claw claw, Intake intake){
        this.CLAW = claw;
        this.INTAKE = intake;

        addRequirements(claw);
        addRequirements(intake);
    }

    public void initialize()
    {
        //CLAW.toPosition(0);
       INTAKE.toSetpoint(2);
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
