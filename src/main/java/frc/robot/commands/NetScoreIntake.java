package frc.robot.commands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;



public class NetScoreIntake extends Command {
    
    Claw CLAW;
    Intake INTAKE;
    Elevator ELEVATOR;

    public NetScoreIntake(Claw claw, Intake intake, Elevator elevator)
    {
        this.CLAW = claw;
        this.INTAKE = intake;
        this.ELEVATOR = elevator;

        addRequirements(claw);
        addRequirements(intake);
        addRequirements(elevator);
       
    }

    public void execute()
    {
        isFinished();
    }

    public boolean isFinished()
    {

        CLAW.spinRollers(100);
        if (!CLAW.pieceDetected())
        {
            return true;
        }
        

        return false;
    }

    

}
