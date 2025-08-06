// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReefTagRelativeAlgae extends Command {
  private HolonomicDriveController holoController = new HolonomicDriveController(
    new PIDController(4, 0, 0), new PIDController(4, 0, 0), new ProfiledPIDController(0.3, 0, 0, new TrapezoidProfile.Constraints(3.14 / 2, 3.14 / 4)));

  private double ySpeed;
  private CommandSwerveDrivetrain drivetrain;
  private SwerveRequest.RobotCentric robotDrive;
  private double tagID;

  private Pose2d robotPose;
  private Pose2d tagPose;
  private Pose2d relativePose;
  private AprilTagFieldLayout field;
  private double[] positions;

  public AlignToReefTagRelativeAlgae(CommandSwerveDrivetrain drive) {
    ySpeed = 0;
    field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    robotDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    this.drivetrain = drive;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    tagID = drivetrain.getLastKnownReefTag();
    robotPose = new Pose2d();
    tagPose = field.getTagPose((int)tagID).get().toPose2d();
    relativePose = new Pose2d();


    holoController.getYController().setSetpoint(0); //0.1 //TX
    holoController.getYController().setTolerance(0.01);
  
    SmartDashboard.putNumber("tagIDDD", tagID);
  }

  @Override
  public void execute()   {
    if (tagID > -1 && LimelightHelpers.getTV("limelight")) {
      positions = LimelightHelpers.getBotPose_TargetSpace("limelight");
      robotPose = drivetrain.getPose();
      relativePose = robotPose.relativeTo(tagPose);

      //tSpeed = holoController.getThetaController().calculate(positions[4]);
      ySpeed = -holoController.getYController().calculate(positions[0]);

      SmartDashboard.putNumber("yspee", ySpeed);
      SmartDashboard.putNumber("y pos", positions[0]);


      drivetrain.setControl(
        robotDrive.withVelocityY(ySpeed)
      );

    } else {
      drivetrain.setControl(
        robotDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
      );
    }
  }
}