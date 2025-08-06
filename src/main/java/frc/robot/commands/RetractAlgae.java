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
public class RetractAlgae extends Command {
    Claw CLAW;
    Intake INTAKE;

    public RetractAlgae(Claw claw, Intake intake){
        this.CLAW = claw;
        this.INTAKE = intake;

        addRequirements(claw);
        addRequirements(intake);
    }

    public void initialize()
    {
        INTAKE.setRollers(0);
        INTAKE.toSetpoint(3);
    }
    
    public void execute()
    {
        isFinished();
    }

    @Override
    public boolean isFinished() {
        /*if(INTAKE.getPosition() < 2){
            CLAW.customPosition(1.4);
            
            return true;
            //there is a chance that you will need to check for algae and then try to move up to 90 degrees
        }
        return false;*/

        return true;
        
    }       
}