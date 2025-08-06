package frc.robot.commands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;


/* 
 - bind command to button (should bind to trigger (hold to extend, retracts when let go))
 - extends forward (to 2nd position), and runs rollers
 - intakes algae from floor (will be in area between intake and front panel, 1st position) 
  - and then claw comes down to 0 position and sucks in algae
*/
public class IntakeAlgae extends Command {
    Claw CLAW;
    Intake INTAKE;

    public IntakeAlgae(Claw claw, Intake intake){
        this.CLAW = claw;
        this.INTAKE = intake;

        addRequirements(claw);
        addRequirements(intake);
    }

    public void initialize()
    {
        CLAW.toPosition(9);
       INTAKE.toSetpoint(2);
       INTAKE.setRollers(60); 
    }
    
    public void execute()
    {
        isFinished();
    }

    @Override
    public boolean isFinished() {
        return true;
    }       
}