package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class StowAlgaeIntake extends Command{

    Intake INTAKE;

    public StowAlgaeIntake(Intake intake){
        this.INTAKE = intake;
        addRequirements(intake);
    }

    public void initialize()
    {
        INTAKE.toSetpoint(0);
    }
    public boolean isFinished(){
        return true;
    }
}
