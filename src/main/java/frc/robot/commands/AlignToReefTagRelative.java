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

public class AlignToReefTagRelative extends Command {
  private HolonomicDriveController holoController = new HolonomicDriveController(
    new PIDController(7.5, 0, 0), new PIDController(7.5, 0, 0), new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(3.14 / 2, 3.14 / 4)));

  private double xSpeed, ySpeed, tSpeed;
  private boolean isRightScore;
  private CommandSwerveDrivetrain drivetrain;
  private SwerveRequest.RobotCentric robotDrive;
  private double tagID;
  private boolean invert = false;
  public double Freaklin = -0.62; //constant (needed) (for some reason review later)

  private Pose2d robotPose; 
  private Pose2d tagPose;
  private Pose2d relativePose;
  private AprilTagFieldLayout field;
  private double[] positions;

  public AlignToReefTagRelative(boolean isRightScore, CommandSwerveDrivetrain drive, boolean inverted) {
    xSpeed = 0;
    ySpeed = 0;
    tSpeed = 0;
    field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    robotDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    this.invert = inverted;
    this.isRightScore = isRightScore;
    this.drivetrain = drive;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    tagID = drivetrain.getLastKnownReefTag();
    robotPose = new Pose2d();
    tagPose = field.getTagPose((int)tagID).get().toPose2d();
    relativePose = new Pose2d();

    holoController.getThetaController().setGoal(0);
    holoController.getThetaController().setTolerance(0.5);

    if (isRightScore) {
      //y controller controls left and right relative to april tag
      holoController.getYController().setSetpoint(0.12); //0.1 //TX
      holoController.getXController().setSetpoint(Freaklin + 0.046); //TZ
    } else {
      holoController.getYController().setSetpoint(-0.13); //-0.13
      holoController.getXController().setSetpoint(Freaklin + 0.046);
    }
      holoController.getXController().setTolerance(-0.01);
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
      xSpeed = holoController.getXController().calculate(positions[2]);
      ySpeed = -holoController.getYController().calculate(positions[0]);
      tSpeed = holoController.getThetaController().calculate(positions[4]);

      SmartDashboard.putNumber("yspeed", ySpeed);
      SmartDashboard.putNumber("y pos", positions[0]);
      SmartDashboard.putNumber("pos 1", positions[1]);
      SmartDashboard.putNumber("Tspeed", tSpeed);
      SmartDashboard.putNumber("pos 4", positions[4]);
      SmartDashboard.putNumber("xspeed", xSpeed);
      SmartDashboard.putNumber("x pos", positions[2]);

      drivetrain.setControl(
        robotDrive.withVelocityY(ySpeed).withVelocityX(xSpeed).withRotationalRate(tSpeed)
      );

    } else {
      drivetrain.setControl(
        robotDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
      );
    }
  }
}