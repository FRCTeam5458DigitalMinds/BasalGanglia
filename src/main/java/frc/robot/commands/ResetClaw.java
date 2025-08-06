package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class ResetClaw extends Command 
{
    Claw CLAW;

    public ResetClaw(Claw claw)
    {
        this.CLAW = claw;
        
        addRequirements(claw);
    }

    public void initialize()
    {

        CLAW.spinRollers(0);

        SmartDashboard.putNumber("Claw Position Ticks: ", CLAW.getPosition());
        SmartDashboard.putNumber("TOF Distance", CLAW.getTOFDistance());

        isFinished();
    }

    public boolean isFinished()
    {
        return true;
    }
}
