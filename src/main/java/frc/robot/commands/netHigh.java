package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class netHigh extends Command 
{
    Claw CLAW;
    Elevator ELEVATOR;
    Intake INTAKE;

    public netHigh(Claw claw, Elevator elevator, Intake intake) 
    {
        this.CLAW = claw;
        this.ELEVATOR = elevator;
        this.INTAKE = intake;

        addRequirements(claw);
        addRequirements(elevator);
        addRequirements(intake);
    }

    public void initialize()
    {
        
    }

    public void execute()
    {
        isFinished();
    }

    public boolean isFinished()
    {
        if (CLAW.getPosition() > 13.25) { //13.5
            ELEVATOR.changeStage6000(14);
        }

        if (ELEVATOR.getPosition() > 35.5) //35
        {
            CLAW.toPosition12000(7); //Original 6
            return true;
        }
        
        return false;
    }
}