package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorMove extends Command {
    Elevator ELEVATOR;
    //setPoint passed through command to tell elevator where to move
    int SETPOINT;

    public ElevatorMove(Elevator elevator, int setPoint) {
        this.ELEVATOR = elevator;
        this.SETPOINT = setPoint;

        addRequirements(elevator);
    }

    /*
    setpoint chart
    0 - starting position (fully down)
    1 - ? get from mech soon
    */
    public void initialize() {
        ELEVATOR.changeStage(SETPOINT);
        
        isFinished();
    }

    public boolean isFinished() {
        return true;
    }
}