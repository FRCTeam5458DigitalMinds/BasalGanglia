package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class ReefScoring extends Command 
{
    Claw CLAW;
    Elevator ELEVATOR;
    int LEVEL;
    Intake INTAKE;

    public ReefScoring(Claw claw, Elevator elevator, int level, Intake intake) 
    {
        this.CLAW = claw;
        this.ELEVATOR = elevator;
        this.LEVEL = level;
        this.INTAKE = intake;

        addRequirements(claw);
        addRequirements(elevator);
        addRequirements(intake);

    }

    public void initialize()
    {
        if (LEVEL < 10 && LEVEL != 1 && LEVEL != 0){
            
            CLAW.toPosition(6);
        }
        else if (LEVEL == 1) {
            CLAW.toPosition(1);
        }
        else if (LEVEL == 0) {
            CLAW.customPosition(1.4);
        }
        else {
                CLAW.toPosition(8);
        }
    }

    public void execute()
    {
        SmartDashboard.putNumber("Claw Pos", CLAW.getPosition());
        isFinished();
    }

    public boolean isFinished()
    {
        if (CLAW.getPosition() > 10 && LEVEL != 1)
        {
            if (LEVEL > 10) {
                ELEVATOR.changeStage(LEVEL);
            } else {
                if (LEVEL == 6 && CLAW.getPosition() > 14)
                {
                    return true;
                }
                else
                {
                    ELEVATOR.changeStage(LEVEL);
                }
            }

            if (Math.abs(ELEVATOR.getV()) < .01)
            {
                if (LEVEL == 4)
                {
                    return ELEVATOR.getPosition() > 37;
                }
                SmartDashboard.putNumber("getV", ELEVATOR.getV());
                return true;
            }
        }

        if (LEVEL == 1 && CLAW.getPosition() > 9)
        {
            return true;
        }
        if (LEVEL == 0 && CLAW.getPosition() > 1.2)
        {
            INTAKE.customPosition(-0.7);
            return true;
        }
        return false;
    }
}