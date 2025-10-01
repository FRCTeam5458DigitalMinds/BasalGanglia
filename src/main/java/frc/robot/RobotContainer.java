package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ReefScoring;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.RemoveAlgae;
import frc.robot.commands.StowElevatorClaw;
import frc.robot.commands.autoClaw;
import frc.robot.commands.netHigh;
import frc.robot.commands.poseState;
import frc.robot.commands.ResetClaw;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.RetractAlgae;
import frc.robot.commands.StowAlgae;
import frc.robot.commands.StowAlgaeIntake;
import frc.robot.commands.testAutoClaw;
import frc.robot.commands.NetScoreIntake;
import frc.robot.commands.ProcessorScore;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.commands.AlignToReefTagRelativeAlgae;
import frc.robot.commands.Eject;
import frc.robot.commands.EjectAlgae;
import frc.robot.commands.Groundsetup;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class RobotContainer {
    //We comment and we dont change - Lambert L. Li

    //Prevents motors from running hot
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    //Never used
    private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //private final CommandXboxController driverController = new CommandXboxController(1);
    private final CommandXboxController operatorController = new CommandXboxController(0);
    private final CommandXboxController driverController = new CommandXboxController(1);

    //Used to determine starting side
    private final int blueConst = 1;
    private final int redConst = -1;
    private int sideConst = -1;

    //Controls the acceleration of the drivetrain
    private final PIDController translatePID = new PIDController(5, 0, 0.01);
    private final PIDController strafePID = new PIDController(5, 0, 0.01);
    private final PIDController rotatePID = new PIDController(0.4, 0, 0.01);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //Creates subsystem objects
    private final Elevator ELEVATOR = new Elevator();
    private final Intake INTAKE = new Intake();
    private final Claw CLAW = new Claw();

    private final SendableChooser<Command> autoChooser2;

    public RobotContainer() {
        NamedCommands.registerCommand("AutoAlignLeft", 
            (drivetrain.applyRequest(() -> 
                drive.withVelocityX(translatePID.calculate(drivetrain.getPose().getX(), drivetrain.getLeftTargetPose().getX()))
                .withVelocityY(strafePID.calculate(drivetrain.getPose().getY(), drivetrain.getLeftTargetPose().getY()))
                .withRotationalRate(rotatePID.calculate(drivetrain.getPose().getRotation().getDegrees(), drivetrain.getLeftTargetPose().getRotation().getDegrees() - 1))
            ))
        );

        //Switches the starting point to correct side
        if (DriverStation.getAlliance().get() == Alliance.Red)
        {
            sideConst *= redConst;
        }

        //Pathplanner Auto Commands
        NamedCommands.registerCommand("Pose", new poseState(drivetrain));
        NamedCommands.registerCommand("L4", new ReefScoring(CLAW, ELEVATOR, 4, INTAKE).andThen(new Eject(CLAW, 4)));
        NamedCommands.registerCommand("Stow", new StowElevatorClaw(ELEVATOR, CLAW));
        NamedCommands.registerCommand("L1", new ReefScoring(CLAW, ELEVATOR, 1, INTAKE).andThen(new Eject(CLAW, 1)).andThen(new StowElevatorClaw(ELEVATOR, CLAW)));
        NamedCommands.registerCommand("Intake", new autoClaw(CLAW));

        //Make an auto chooser on the smart dashboard
        autoChooser2 = AutoBuilder.buildAutoChooser();
        
        SmartDashboard.putData("Auto Chooser", autoChooser2);
        //set the button bindings
        configureBindings();
    }

    private void configureBindings() {
        //DRIVE COMMANDS
        //Drivetrain always checks the joysticks to drive and rotate
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //Auto Align for coral! yipee
        driverController.leftBumper().and(() -> CLAW.pieceDetected()).and(() -> !operatorController.start().getAsBoolean()).whileTrue(
            new AlignToReefTagRelative(false, drivetrain, true)
        );

        //Auto Align for Algae! yipee
        driverController.leftBumper().and(() -> !CLAW.pieceDetected()).and(() -> !operatorController.start().getAsBoolean()).whileTrue(
            new AlignToReefTagRelativeAlgae(drivetrain)
        );
        driverController.rightBumper().and(() -> !CLAW.pieceDetected()).and(() -> !operatorController.start().getAsBoolean()).whileTrue(
            new AlignToReefTagRelativeAlgae(drivetrain)
        );
        /* Work in progress
        operatorController.leftBumper().and(() -> CLAW.pieceDetected()).and(operatorController.start()).whileTrue(
            new AlignToReefTagRelative(false, drivetrain, false)
        );
        */
        //operatorController.leftBumper().and(() -> !CLAW.pieceDetected()).whileTrue(
        //    drivetrain.applyRequest(() -> 
        //        robotDrive.withVelocityX(-translatePID.calculate(drivetrain.getPose().getX(), drivetrain.getCenterTargetPose().getX()))
        //        .withVelocityY(-strafePID.calculate(drivetrain.getPose().getY(), drivetrain.getCenterTargetPose().getY()))
        //        //.withRotationalRate(rotatePID.calculate(drivetrain.getYaw(), 1))
        //    )
        //);
        //More Auto Align. yipee
        driverController.rightBumper().and(() -> !driverController.start().getAsBoolean()).whileTrue(
            new AlignToReefTagRelative(true, drivetrain, true)
        );
        /* Work in progress
        operatorController.rightBumper().and(operatorController.start()).whileTrue(
            new AlignToReefTagRelative(true, drivetrain, false)
        );
        */

        //operatorController.rightBumper().and(operatorController.start()).whileTrue(
        //    drivetrain.applyRequest(() -> 
        //        robotDrive.withVelocityX(-translatePID.calculate(drivetrain.getPose().getX(), drivetrain.getRightTargetPose().getX()))
        //        .withVelocityY(-strafePID.calculate(drivetrain.getPose().getY(), drivetrain.getRightTargetPose().getY())))
        //);

       // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
       // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
       // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
       // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
       // driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);

        //Claw will always run this command when not running any other
        CLAW.setDefaultCommand(new testAutoClaw(CLAW));

        //driverController.povLeft().onTrue(new StowElevatorClaw(ELEVATOR, CLAW));

        //operatorController.rightTrigger().whileTrue(new IntakeAlgae(CLAW, INTAKE)).onFalse(new StowAlgae(CLAW, INTAKE));
        
        //scores algae into the net
        driverController.povUp().onTrue(new netHigh(CLAW, ELEVATOR, INTAKE,1).andThen(new NetScoreIntake(CLAW, INTAKE, ELEVATOR)));
        //operatorController.povRight().onTrue(new ProcessorScore(INTAKE));
        //scores coral to L1
        operatorController.a().onTrue(new ReefScoring(CLAW, ELEVATOR, 1, INTAKE).andThen(new Eject(CLAW, 1)).andThen(new StowElevatorClaw(ELEVATOR, CLAW)));
        //scores coral to L2
        operatorController.x().and(() -> CLAW.pieceDetected()).and(() -> !operatorController.rightBumper().getAsBoolean()).and(() -> !operatorController.leftBumper().getAsBoolean()).onTrue(new ReefScoring(CLAW, ELEVATOR, 2, INTAKE));
        //takes algae off the reef
        operatorController.x().and(() -> !CLAW.pieceDetected()).and(() -> !operatorController.rightBumper().getAsBoolean()).and(() -> !operatorController.leftBumper().getAsBoolean()).onTrue(new ReefScoring(CLAW, ELEVATOR, 12, INTAKE).andThen(new RemoveAlgae(CLAW,INTAKE,2, 95)));
        //operatorController.axisGreaterThan(2, 0.1).onTrue(new ResetGyro(drivetrain));
        //shoots coral from claw
        operatorController.axisGreaterThan(3, 0.05).onTrue(new Eject(CLAW, 2));

        //picks up algae from floor
        driverController.axisGreaterThan(2, 0.05).onTrue(new IntakeAlgae(CLAW, INTAKE));
        //collect algae into robot
        driverController.axisGreaterThan(2, 0.05).onFalse(new RetractAlgae(CLAW, INTAKE));//.andThen(new ReefScoring(CLAW, ELEVATOR, 0)).andThen(new RemoveAlgae(CLAW,INTAKE,0)));
        //emergency remove algae (unreliable)
        driverController.axisGreaterThan(3, 0.05).onTrue(new EjectAlgae(CLAW, INTAKE));
        //redundant
        driverController.axisGreaterThan(3, 0.05).onFalse(new RetractAlgae(CLAW, INTAKE));
        //puts algae into claw
        driverController.a().onTrue(new ReefScoring(CLAW, ELEVATOR, 0, INTAKE).andThen(new RemoveAlgae(CLAW,INTAKE,0, 100)));
        //moves claw up sloooowwwwly
        driverController.y()/*.and(() -> CLAW.pieceDetected())*/.onTrue(new Groundsetup(CLAW, INTAKE).andThen(new RetractAlgae(CLAW, INTAKE)));
        //spits out algae (used for processor)
        driverController.x().onTrue(new ProcessorScore(INTAKE));
        //scores coral to L3
        operatorController.y().and(() -> CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 3, INTAKE));
        //collects higher algae from reef
        operatorController.y().and(() -> !CLAW.pieceDetected()).onTrue(new ReefScoring(CLAW, ELEVATOR, 13, INTAKE).andThen(new RemoveAlgae(CLAW,INTAKE,3,95)));
        //scores coral to L4
        operatorController.b().onTrue(new ReefScoring(CLAW, ELEVATOR, 4, INTAKE));
        //resets elevater, claw, and floor intake
        operatorController.povLeft().onTrue(new StowElevatorClaw(ELEVATOR, CLAW));
        driverController.povLeft().onTrue(new StowElevatorClaw(ELEVATOR, CLAW).andThen(new StowAlgaeIntake(INTAKE)));
        //resets gyro to 0
        driverController.b().onTrue(new ResetGyro(drivetrain));
        
        //ghost code
        //driverController.y().onTrue()  Work in progress. Score net from ground.
    
        
    }

    public Command getAutonomousCommand() {
        return autoChooser2.getSelected();
    }
}