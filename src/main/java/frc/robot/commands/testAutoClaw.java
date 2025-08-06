package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class testAutoClaw extends Command 
{
    Claw CLAW;

    public testAutoClaw (Claw claw)
    {
        this.CLAW = claw;
        
        addRequirements(claw);
    }

    public void execute()
    {
        if (CLAW.pieceDetected() == true && CLAW.getPosition() < 1)
        {
            SmartDashboard.putBoolean("Piece Detected", CLAW.pieceDetected());
            CLAW.spinRollers(0);
        }

        if (CLAW.pieceDetected() == false && CLAW.getPosition() < 1)
        {   
            CLAW.spinRollers(30);
        }

        SmartDashboard.putNumber("Claw Position Ticks: ", CLAW.getPosition());
        SmartDashboard.putNumber("TOF Distance", CLAW.getTOFDistance());
    }
}
