package frc.robot.commands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class StowElevatorClaw extends Command {

    Elevator ELEVATOR;
    Claw CLAW;


    public StowElevatorClaw(Elevator elevator, Claw claw)
    {
        this.ELEVATOR = elevator;
        this.CLAW = claw;
        
        addRequirements(elevator);
        addRequirements(claw);
    }
    
    public void initialize()
    {
        CLAW.spinRollers(0);
        ELEVATOR.changeStage(0);
    }

    public void execute()
    {
        SmartDashboard.putNumber("Claw Pos", CLAW.getPosition());

        isFinished();
    }
    

    public boolean isFinished()
    {
        if (ELEVATOR.getPosition() < 1)
        {
            CLAW.toPosition(0);
            return true;
        }
        return false;
    }
}


