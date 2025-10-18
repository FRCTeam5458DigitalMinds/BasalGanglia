package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;
import java.util.function.Supplier;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.LimelightHelpers;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    boolean hasTarget;
    double tx;
    double ty;
    double ta;
    long tag_ID;
    double txnc;
    double tync;
    double reefTagHeightInches = 0.307975; 
  
    private double[] tagPoseRotation = {120, 179, 60, 0, -60, -120};

    private int[] redReef = {6, 7, 8, 9, 10, 11};
    private int[] blueReef = {17, 18, 19, 20, 21, 22};

    private LimelightHelpers.PoseEstimate limelightMeasurement = new LimelightHelpers.PoseEstimate();

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    private int lastKnownReefTag = 7;

    //creates an image in the field
    private final Field2d m_field = new Field2d();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final Pigeon2 pigeon = new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id);

    private final SwerveDrivePoseEstimator m_poseEstimator =
    new SwerveDrivePoseEstimator(
        getKinematics(),
        pigeon.getRotation2d(),
        getState().ModulePositions,
        new Pose2d(),
        VecBuilder.fill(1.5, 1.5, Units.degreesToRadians(5)),
        VecBuilder.fill(1.5, 1.5, Units.degreesToRadians(30)));

        //gets position based of apriltags
    public Pose2d getPose()
    {
        return m_poseEstimator.getEstimatedPosition();
    }

    public double getTagRotation()
    {
        if (getLastKnownReefTag() < 12)
        {
            return Math.toRadians(tagPoseRotation[getLastKnownReefTag() - 6]);
        }
        SmartDashboard.putNumber("radians", Math.toRadians(tagPoseRotation[getLastKnownReefTag() - 17]));
        return Math.toRadians(tagPoseRotation[getLastKnownReefTag() - 17]);
    }

    public int getLastKnownReefTag()
    {        
        int currentTag = (int)LimelightHelpers.getFiducialID("limelight");
        if (currentTag > 0)
        {
            lastKnownReefTag = currentTag;
        }
        SmartDashboard.putNumber("cur id", currentTag);
        SmartDashboard.putNumber("last id", lastKnownReefTag);

        return lastKnownReefTag;
    }

    public void resetGyro()
    {
        pigeon.reset();
        SmartDashboard.putNumber("Yaw", pigeon.getYaw().getValueAsDouble());
        //pigeon.setYaw(0);
    }
    
    public Pose2d getLeftTargetPose()
    {
        Pose2d TAG = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(getLastKnownReefTag()).get().toPose2d();
        Transform2d LEFTPOSE = new Transform2d(new Pose2d(), new Pose2d(0.12, -.25, new Rotation2d().fromDegrees(180)));
        Pose2d TARGETPOSELEFT = TAG.transformBy(LEFTPOSE);
        //Pose2d TARGETPOSERIGHT = TAG.transformBy(RIGHTPOSE);
        //Pose2d ALGAEPOSE = TAG.transformBy(ALGAE);

        SmartDashboard.putNumber("ry", TARGETPOSELEFT.getY());
        SmartDashboard.putNumber("rx", TARGETPOSELEFT.getX());

        return TARGETPOSELEFT;
    }

    public Pose2d getCenterTargetPose()
    {
        Pose2d TAG = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(getLastKnownReefTag()).get().toPose2d();
        Transform2d ALGAE = new Transform2d(new Pose2d(), new Pose2d(-0.1, 0, new Rotation2d(-1)));
        Pose2d TARGETPOSELEFT = TAG.transformBy(ALGAE);

        

        return TARGETPOSELEFT;
    }

    public Pose2d getRightTargetPose()
    {
        Pose2d TAG = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(getLastKnownReefTag()).get().toPose2d();
        SmartDashboard.putNumber("robot rot", getRotation().getDegrees());
        //Transform2d RIGHTPOSE = new Transform2d(new Pose2d(), new Pose2d(((-0.27*Math.cos(getTagRotation())) + (0.15*Math.sin(getTagRotation()))), -((-0.27*Math.sin(getTagRotation())) - (0.16*Math.cos(getTagRotation()))), new Rotation2d().fromDegrees(180)));

        Transform2d RIGHTPOSE  = new Transform2d(new Pose2d(), new Pose2d(0.12, -0.25, new Rotation2d().fromDegrees(180)));
        Pose2d TARGETPOSELEFT = TAG.transformBy(RIGHTPOSE);
        SmartDashboard.putNumber("tag rot", TARGETPOSELEFT.getRotation().getDegrees());

        
        SmartDashboard.putNumber("ry", TARGETPOSELEFT.getY());
        SmartDashboard.putNumber("rx", TARGETPOSELEFT.getX());

        return TARGETPOSELEFT;
    }

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );


    public void updateLimelightTracking() 
    {
        tx = LimelightHelpers.getTX("limelight");  // Horizontal offset from crosshair to target in degrees
        ty = LimelightHelpers.getTY("limelight");  // Vertical offset from crosshair to target in degrees
        ta = LimelightHelpers.getTA("limelight");  // Target area (0% to 100% of image)
        hasTarget = LimelightHelpers.getTV("limelight"); // Do you have a valid target?

        txnc = LimelightHelpers.getTXNC("limelight");  // Horizontal offset from principal pixel/point to target in degrees
        tync = LimelightHelpers.getTYNC("limelight");
    }

    public boolean hasValidTarget()
    {
        updateLimelightTracking();
        return hasTarget;
    }
    
    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        setFiducialIds();
        SmartDashboard.putData("Field", m_field);
        configureAutoBuilder();
    }

    public void resetPoseEstimator()
    {
        m_poseEstimator.resetPose(getState().Pose);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        SmartDashboard.putData("Field", m_field);
        configureAutoBuilder();
    }

    public void resetPOSE()
    {
        resetPose(new Pose2d());
    }
    
    private void configureAutoBuilder() {
        try {
            
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getPose(),   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(1, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /*public Command generatePath(Pose2d targetPose)
    {
        //PathConstraints constraints = new PathConstraints(0.5, 0.25, .05 * Math.PI, .1 * Math.PI); // The constraints for this path.
        PathConstraints constraints = new PathConstraints(3, 2, 100, 278); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // reate the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(getPose(), targetPose),
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        ); 
        path.preventFlipping = true;
        
        return AutoBuilder.followPath(path);
    } */
    /* 
    public Command autoAlignCommand(Pose2d waypoint)
    {
        PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0);

        Command pathfinding = AutoBuilder.pathfindToPose(
            waypoint,
            constraints,
            0.0
        );

        return pathfinding;
    } */
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    private void setFiducialIds() {
        int[] validIds = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIds);
    }

    public double getYaw()
    {
        return this.getPigeon2().getYaw().getValueAsDouble();
    }

    @Override
    public void periodic() {
        updateOdometry();

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Rotation2d getRotation()
    {
        return getState().Pose.getRotation();
    }

    public void updateOdometry() {
        m_field.setRobotPose(getPose());
        SmartDashboard.putNumber("field pose", m_field.getRobotPose().getX());

        m_poseEstimator.update(
            getRotation(),
            getState().ModulePositions);

        SmartDashboard.putNumber("poseX", m_poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("state X", getState().Pose.getX());
        SmartDashboard.putNumber("state rot", getState().Pose.getRotation().getRotations());


        SmartDashboard.putNumber("poseY", m_poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("robot rot func", getRotation().getRotations());
        SmartDashboard.putNumber("pose estiator rot", m_poseEstimator.getEstimatedPosition().getRotation().getRotations());

        boolean doRejectUpdate = false;
        
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
          if(Math.abs(pigeon.getAccelerationX().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
          {
            doRejectUpdate = true;
          }
          if (mt2 != null)
          {
          if(mt2.tagCount == 0)
          {
            doRejectUpdate = true;
          }
          if(!doRejectUpdate)
          {
            SmartDashboard.putString("state", "not reject");
            
            SmartDashboard.putNumber("mt2 X", mt2.pose.getX());
            SmartDashboard.putNumber("mt2 Y", mt2.pose.getY());

            SmartDashboard.putNumber("poseY", m_poseEstimator.getEstimatedPosition().getY());
            LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(5,5,10));
            m_poseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);


          } else {
            SmartDashboard.putString("state", "reject");
          }
        }
      }
}
