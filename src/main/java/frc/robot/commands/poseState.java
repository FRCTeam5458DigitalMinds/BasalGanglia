package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class poseState extends Command {
    CommandSwerveDrivetrain DRIVETRAIN;

    public poseState(CommandSwerveDrivetrain drivetrain) 
    {
        this.DRIVETRAIN = drivetrain;
        addRequirements(drivetrain);
    }

    public boolean isFinished()
    {
        DRIVETRAIN.resetPoseEstimator();
        return true;
    }
}