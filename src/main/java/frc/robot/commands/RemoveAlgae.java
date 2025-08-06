package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;

public class RemoveAlgae extends Command 
{
    private final Claw CLAW;
    private final Intake INTAKE;
    private final int LEVEL;

    public RemoveAlgae(Claw claw, Intake intake,int level)
    {
        this.CLAW = claw;
        this.INTAKE = intake;
        this.LEVEL = level;

        addRequirements(claw);
        addRequirements(intake);
    }

    public void initialize()
    {
        CLAW.spinRollers(-100);
        INTAKE.setRollers(0); //30 or 0 for processor
    }

    public void execute()
    {
        isFinished();            
    }

    public boolean isFinished()
    {
        SmartDashboard.putNumber("TOF", CLAW.getTOFDistance());
        if(CLAW.algaeDetected())
        {
            CLAW.spinRollers(-95);
            if (LEVEL == 0){
                CLAW.toPosition(9);
                INTAKE.setRollers(0);
                return true;
            }
            else{
                INTAKE.setRollers(0);
                return true;
            }
            
        }
        return false;
    }
}
