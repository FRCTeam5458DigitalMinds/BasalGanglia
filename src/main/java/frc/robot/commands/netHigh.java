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
    int evilHorse;

    public netHigh(Claw claw, Elevator elevator, Intake intake, int goodHorse) 
    {
        this.CLAW = claw;
        this.ELEVATOR = elevator;
        this.INTAKE = intake;
        this.evilHorse = goodHorse;

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
        if (evilHorse == 1){
            INTAKE.toSetpoint(2);//6000
        }
        if (INTAKE.getPosition() < -5){
            CLAW.toPosition6000(10);
        }
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