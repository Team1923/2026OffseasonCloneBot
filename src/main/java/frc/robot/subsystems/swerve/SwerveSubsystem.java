package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.StateHandler;
import org.littletonrobotics.junction.Logger;

/**
 * Command-based swerve subsystem. Hardware access goes through {@link SwerveIO} so the drivetrain
 * can be logged and replayed with AdvantageKit.
 */
public class SwerveSubsystem extends SubsystemBase {

    private static SwerveSubsystem swerveSubsystem;

    public static synchronized SwerveSubsystem getInstance() {
        if (swerveSubsystem == null) {
            swerveSubsystem = new SwerveSubsystem(
                    switch (StateHandler.getInstance().robotMode) {
                        case REAL -> new SwerveIO.Real();
                        case SIM -> new SwerveIO.Sim();
                        case REPLAY -> new SwerveIO() {};
                    });
        }
        return swerveSubsystem;

    }

    public static enum SwerveStates {
        FIELD_CENTRIC,
        ROBOT_CENTRIC,
        FACING_HEADING
    }

    private final SwerveIO io;
    private final SwerveIO.SwerveIOInputs inputs = new SwerveIO.SwerveIOInputs();

    private final Alert gyroDisconnectedAlert = new Alert("Swerve gyro disconnected", AlertType.kError);
    private final Alert[] moduleDisconnectedAlerts = new Alert[SwerveIO.MODULE_COUNT];

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private final StateHandler stateHandler = StateHandler.getInstance();

    /** Swerve request to apply during robot-centric path following */
    private static final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    private SwerveSubsystem(SwerveIO io) {
        this.io = io;
        for (int i = 0; i < SwerveIO.MODULE_COUNT; i++) {
            moduleDisconnectedAlerts[i] = new Alert(
                    "Swerve module " + i + " hardware disconnected", AlertType.kError);
        }

        configureAutoBuilder();

        PathPlannerLogging.setLogActivePathCallback(
                poses -> Logger.recordOutput("Swerve/Auto/ActivePath", poses.toArray(new Pose2d[0])));
        PathPlannerLogging.setLogTargetPoseCallback(
                pose -> Logger.recordOutput("Swerve/Auto/TargetPose", pose));
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    this::getPose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    this::getSpeeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(5, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(5, 0, 0)), // real 7
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> stateHandler.isRedAlliance,
                    this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
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

    /** Applies a CTRE swerve request to the modules. */
    public void setControl(SwerveRequest request) {
        io.setControl(request);
        Logger.recordOutput("Swerve/Request", request.getClass().getSimpleName());
    }

    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
        Logger.recordOutput("Swerve/ResetPose", pose);
    }

    /**
     * Fuses a vision pose into the estimator.
     *
     * @param timestampSeconds capture time in CTRE's time base
     */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        io.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public Pose2d getPose() {
        return inputs.pose;
    }

    public ChassisSpeeds getSpeeds() {
        return inputs.speeds;
    }

    public double getYawVelocityDegreesPerSecond() {
        return inputs.gyroYawVelocityDegreesPerSecond;
    }

    /** Requests heading lock while leaving translation under the default drive state machine. */
    public Command faceTargetCommand(Supplier<Pose2d> targetSupplier) {
        return Commands.run(
                        () -> {
                            Pose2d pose = getPose();
                            Pose2d target = targetSupplier.get();
                            stateHandler.desiredSwerveHeading =
                                    target.getTranslation().minus(pose.getTranslation()).getAngle();
                            stateHandler.swerveState = SwerveStates.FACING_HEADING;
                        })
                .finallyDo(interrupted -> stateHandler.swerveState = SwerveStates.FIELD_CENTRIC)
                .withName("SwerveFaceTarget");
    }

    public boolean isFacing(Pose2d target) {
        Pose2d pose = getPose();
        Rotation2d desiredHeading = target.getTranslation().minus(pose.getTranslation()).getAngle();
        double errorDegrees = desiredHeading.minus(pose.getRotation()).getDegrees();
        return Math.abs(errorDegrees) < Constants.SwerveConstants.HEADING_TOLERANCE_DEGREES;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve", inputs);

        gyroDisconnectedAlert.set(!inputs.gyroConnected);
        for (int i = 0; i < SwerveIO.MODULE_COUNT; i++) {
            moduleDisconnectedAlerts[i].set(
                    !(inputs.driveConnected[i] && inputs.steerConnected[i] && inputs.encoderConnected[i]));
        }

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            io.setOperatorPerspectiveForward(
                    stateHandler.isRedAlliance
                            ? kRedAlliancePerspectiveRotation
                            : kBlueAlliancePerspectiveRotation);
            m_hasAppliedOperatorPerspective = true;
        }

    }

    public double getCurrentSpeed() {
        return new Translation2d(getSpeeds().vxMetersPerSecond, getSpeeds().vyMetersPerSecond)
                .getNorm();
    }

    // uses the current vector direction of velocity and the desired ending location
    // to find the ideal starting heading to approach the point
    public Rotation2d getIdealApproachVelocityHeadingTo(Pose2d waypoint) {

        // If we are going slow, we can generate the shortest straightline path,
        // regardless of where we are currently facing
        if (this.getCurrentSpeed() < 0.25) {
            Translation2d diffInTranslation = waypoint.getTranslation().minus(getPose().getTranslation());

            return (diffInTranslation.getNorm() < 0.01) // if there is a very small difference between their
                                                        // translations
                    ? waypoint.getRotation() // just go straight towards the waypoint's heading
                    : diffInTranslation.getAngle(); // otherwise, follow the heading difference between us and the reef
        }

        // otherwise, if we are here, we need to velocity compensate

        // CTRE reports robot-relative speeds; rotate into the field frame the path is built in.
        Rotation2d robotRelativeHeading = new Rotation2d(getSpeeds().vxMetersPerSecond,
                getSpeeds().vyMetersPerSecond);

        return robotRelativeHeading.plus(getPose().getRotation());
    }

    public Command onTheFlyPath(Pose2d waypoint) {

        // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        // getPose(),
        // waypoint
        // );

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(getPose().getTranslation(), getIdealApproachVelocityHeadingTo(waypoint)),
                waypoint);

        // PathPlannerPath generated = new PathPlannerPath(
        // waypoints,
        // Constants.SwerveConstants.pathingConstraints,
        // null,
        // new GoalEndState(0, waypoint.getRotation()));

        PathPlannerPath generated = new PathPlannerPath(
                waypoints,
                Constants.SwerveConstants.PATHING_CONSTRAINTS,
                new IdealStartingState(getCurrentSpeed(), getPose().getRotation()),
                new GoalEndState(0, waypoint.getRotation()));

        generated.preventFlipping = true;

        return AutoBuilder.followPath(generated);
    }
}
