package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;

public class Groundsetup extends Command{
    Claw CLAW;
    Intake INTAKE;

    public Groundsetup(Claw claw, Intake intake){
        this.CLAW = claw;
        this.INTAKE = intake;

        addRequirements(claw);
        addRequirements(intake);
    }

    public void initialize()
    {
        if(CLAW.algaeDetected()){
            INTAKE.toSetpoint(2);
        }
    }
    
    public void execute()
    {
        isFinished();
    }

    @Override
    public boolean isFinished() {
        if (INTAKE.getPosition() < -12){
            CLAW.toPosition6000(10);
            return true;
        }
        return false;
    }       

}
