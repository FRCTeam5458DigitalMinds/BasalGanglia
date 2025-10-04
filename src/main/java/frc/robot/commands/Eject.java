package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class Eject extends Command 
{
    Claw CLAW;
    int LEVEL;

    public Eject(Claw claw, int level) 
    {
        this.CLAW = claw;
        this.LEVEL = level;

        addRequirements(claw);
    }
    //sets the roller speed for every level except L1
    public void initialize()
    {
        if (LEVEL != 1)
        {
            SmartDashboard.putNumber("Claw Pos", CLAW.getPosition());
            //CLAW.toPosition(LEVEL);
        }
        else
        {
            CLAW.spinRollers(30);
        }
    }

    public void execute()
    {
        SmartDashboard.putNumber("claw spin", CLAW.getSpin());
        SmartDashboard.putNumber("claw V", CLAW.getV());

        isFinished();            
    }

    public boolean isFinished()
    {
        //resets claw rollers
        if (Math.abs(CLAW.getV()) < 0.05 && LEVEL != 1)
        {
            SmartDashboard.putBoolean("eject done", false);
            SmartDashboard.putNumber("tof", CLAW.getTOFDistance());
            SmartDashboard.putNumber("claw V", CLAW.getV());

            CLAW.spinRollers(-100);
            if (!(CLAW.getTOFDistance() < 0.08) || CLAW.TOFdetect() == false)
            {
                SmartDashboard.putNumber("tof", CLAW.getTOFDistance());
                SmartDashboard.putBoolean("eject done", true);
                return true;
            }
        }
        //finish L1 shooting
        if (LEVEL == 1 && !CLAW.TOFdetect())
        {
            return true;
        }
        return false;
    }
}