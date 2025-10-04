package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class autoClaw extends Command 
{
    Claw CLAW;

    public autoClaw (Claw claw)
    {
        this.CLAW = claw;
        
        addRequirements(claw);
    }

    public void execute()
    {
        isFinished();
    }

    public boolean isFinished()
    {
        //run motors until there's a coral
        if (CLAW.pieceDetected() == true && CLAW.getPosition() < 1)
        {
            SmartDashboard.putBoolean("Piece Detected", CLAW.pieceDetected());
            CLAW.spinRollers(0);
            return true;
        }

        if (CLAW.pieceDetected() == false && CLAW.getPosition() < 1)
        {
            CLAW.spinRollers(30);
        }
        return false;
    }
}
