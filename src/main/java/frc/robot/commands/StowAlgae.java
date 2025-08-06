package frc.robot.commands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class StowAlgae extends Command {
    Claw CLAW;
    Intake INTAKE;

    public StowAlgae(Claw claw, Intake intake){
        this.CLAW = claw;
        this.INTAKE = intake;

        addRequirements(claw);
        addRequirements(intake);
    }

    public void initialize()
    {
        INTAKE.setRollers(0);
        INTAKE.toSetpoint(1);
    }
    
    public void execute()
    {
        isFinished();
    }

    @Override
    public boolean isFinished() {
        if (INTAKE.getV() < 0.02) {
            return true;
        }
        return false;
    }       
}