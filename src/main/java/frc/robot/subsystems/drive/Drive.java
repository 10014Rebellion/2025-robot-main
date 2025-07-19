package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.FieldConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.drive.controllers.HeadingController;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.controllers.GoalPoseChooser;
import frc.robot.subsystems.drive.controllers.GoalPoseChooser.CHOOSER_STRATEGY;
import frc.robot.subsystems.drive.controllers.GoalPoseChooser.SIDE;
import frc.robot.subsystems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.subsystems.drive.controllers.ManualTeleopController;
import frc.robot.subsystems.drive.controllers.HolonomicController;

import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionObservation;
import frc.robot.util.debugging.LoggedTunableNumber;
import frc.robot.util.debugging.SysIDCharacterization;
import frc.robot.util.math.AllianceFlipUtil;
import frc.robot.util.math.GeomUtil;
import frc.robot.util.pathplanner.SwerveSetpoint;
import frc.robot.util.pathplanner.SwerveSetpointGenerator;
import frc.robot.util.swerve.LocalADStarAK;
import frc.robot.util.swerve.SwerveUtils;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/* 
 * This code is a swerve drivebase 
 * Main logic is handled in periodic() function
 */
public class Drive extends SubsystemBase {
    public static enum DriveState {
        // TELEOP AND AUTON CONTROLS
        TELEOP,
        TELEOP_SNIPER,
        POV_SNIPER,
        PROCESSOR_HEADING_ALIGN,
        INTAKE_HEADING_ALIGN,
        REEF_HEADING_ALIGN,
        DRIVE_TO_CORAL,
        DRIVE_TO_ALGAE,
        DRIVE_TO_INTAKE,
        DRIVE_TO_BARGE,
        AUTON, 
        STOP,
        UP,
        DOWN,
        LEFT,
        RIGHT,
        // TESTS
        DRIFT_TEST,
        LINEAR_TEST,
        SYSID_CHARACTERIZATION,
        WHEEL_CHARACTERIZATION
    }

    /* HARDWARE LAYERS */
    private Module[] modules;
    private GyroIO gyro;
    private GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
    private Vision vision;

    /* LOCALIZATION(tracks position and orientation of robot) */
    private Rotation2d robotRotation;
    private SwerveDriveOdometry odometry;
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field = new Field2d();

    /* PATHPLANNER AND SETPOINT GENERATOR(USED IN TELEOP) */
    public static RobotConfig robotConfig;
    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint = new SwerveSetpoint(
        new ChassisSpeeds(), SwerveUtils.zeroStates(), DriveFeedforwards.zeros(4));

    /* STATE OF DRIVEBASE */
    @AutoLogOutput(key="Drive/State")
    private DriveState driveState = DriveState.TELEOP;
    private boolean useGenerator = true;

    /* SETPOINTS */
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    private ChassisSpeeds ppDesiredSpeeds = new ChassisSpeeds();
    private DriveFeedforwards pathPlanningFF = DriveFeedforwards.zeros(4);

    private SwerveModuleState[] prevStates = SwerveUtils.zeroStates();

    /* CONTROLLERS(are used to set chassis speeds) */
    private ManualTeleopController teleopController = new ManualTeleopController();

    private HeadingController headingController = new HeadingController();
    @AutoLogOutput(key="Drive/HeadingController/GoalRotation")
    private Rotation2d goalRotation = new Rotation2d();

    private HolonomicController autoAlignController = new HolonomicController();
    
    @AutoLogOutput(key="Drive/HeadingController/GoalPose")
    private Pose2d goalPose = new Pose2d();

    /* TUNABLE NUMBERS FOR DRIVEBASE CONSTANTS AND TESTS */
    private final static LoggedTunableNumber driftRate = new LoggedTunableNumber("Drive/DriftRate", DriveConstants.kDriftRate);
    private final static LoggedTunableNumber rotationDriftTestSpeedDeg = new LoggedTunableNumber("Drive/DriftRotationTestDeg", 360);
    private final static LoggedTunableNumber linearTestSpeedMPS = new LoggedTunableNumber("Drive/LinearTestMPS", 4.5);

    Debouncer autoAlignTimeout = new Debouncer(0.1, DebounceType.kRising);
    Debouncer autoAlignDelay = new Debouncer(0.1, DebounceType.kRising);

    public Drive(Module[] modules, GyroIO gyro, Vision vision) {
        this.modules = modules;
        this.gyro = gyro;
        this.vision = vision;

        robotRotation = gyroInputs.yawPosition;

        odometry = new SwerveDriveOdometry(kKinematics, getRobotRotation(), getModulePositions());
        poseEstimator = new SwerveDrivePoseEstimator(kKinematics, getRobotRotation(), getModulePositions(), new Pose2d());
        try {
            /* Incase if pathplanner doesn't currently load */
            robotConfig = RobotConfig.fromGUISettings();
        } catch(Exception e) {
            e.printStackTrace();
        }

        setpointGenerator = new SwerveSetpointGenerator(
            robotConfig, // The robot configuration. This is the same config for pathplanner as well
            kMaxAzimuthAngularRadiansPS // The max rotation velocity of a swerve module in radians per second.
        );

        //4.29
        /* Sets up pathplanner to load paths. No need for choreo set-up as its handled internally by Pathplanner */
        /* Refer to getTrajectory() in AutonCommands */
        AutoBuilder.configure(
            this::getPoseEstimate, 
            this::setPose, 
            this::getRobotChassisSpeeds, 
            (speeds, ff) -> {
                driveState = DriveState.AUTON;
                ppDesiredSpeeds = new ChassisSpeeds(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond
                );
                pathPlanningFF = ff;
            }, 
            new PPHolonomicDriveController( kPPTranslationPID, kPPRotationPID ), 
            robotConfig, 
            () -> DriverStation.getAlliance().isPresent() && 
                DriverStation.getAlliance().get() == Alliance.Red, 
            this);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> Logger.recordOutput(
        "Drive/Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> Logger.recordOutput(
        "Drive/Odometry/TrajectorySetpoint", targetPose));

        SmartDashboard.putData(field);

        headingController.setHeadingGoal(() -> goalRotation);
    }

    public Command customFollowPathComamnd(PathPlannerPath path) {
        return new FollowPathCommand(
            path,
            this::getPoseEstimate,
            this::getRobotChassisSpeeds, 
            (speeds, ff) -> {
                ppDesiredSpeeds = speeds;
                pathPlanningFF = ff;
            }, 
            new PPHolonomicDriveController( kPPTranslationPID, kPPRotationPID ), 
            robotConfig, 
            () -> DriverStation.getAlliance().isPresent() && 
                DriverStation.getAlliance().get() == Alliance.Red, 
            this);
    }

    public Command customFollowPathComamnd(PathPlannerPath path, PPHolonomicDriveController drivePID) {
        return new FollowPathCommand(
            path,
            this::getPoseEstimate,
            this::getRobotChassisSpeeds, 
            (speeds, ff) -> {
                driveState = DriveState.AUTON;
                ppDesiredSpeeds = speeds;
                pathPlanningFF = ff;
            }, 
            drivePID, 
            robotConfig, 
            () -> DriverStation.getAlliance().isPresent() && 
                DriverStation.getAlliance().get() == Alliance.Red, 
            this);
    }

    /*
     * If you broke it into functions: updateData -> setChassisSpeeds(data, state) -> runSwerve(speeds) 
     * Periodic Logic: 
     * Data Collection(Modules, gyro, Vision, odometry)
     * Update Controllers use
     * Setting Chassis Speeds based on driveState
     * Teleop sets based of joystick, Autons sets based on pathplanner(ppDesiredSpeeds), etc. etc.
     * runSwerve() function sets the desired state to modules
     * State is set by setDriveState() and related commands which also resets the controllers
    */
    @Override
    public void periodic() {
        ///////////////////// DATA COLLECTION AND UPDATE CONTROLLERS AND ODOMETRY \\\\\\\\\\\\\\\\\\
        /* Modules */
        for (Module module : modules) module.periodic();

        /* GYRO */
        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        if (gyroInputs.connected) robotRotation = gyroInputs.yawPosition;
        else robotRotation = Rotation2d.fromRadians(
            (poseEstimator.getEstimatedPosition().getRotation().getRadians()
            /* D=vt. Uses modules and IK to estimate turn */
            + getRobotChassisSpeeds().omegaRadiansPerSecond * 0.02) 
                /* Scopes result between 0 and 360 */
                % 360.0);

        /* VISION */
        vision.periodic(poseEstimator.getEstimatedPosition(), odometry.getPoseMeters());
        VisionObservation[] observations = vision.getVisionObservations();
        for(VisionObservation observation : observations) {
            if(observation.hasObserved()) poseEstimator.addVisionMeasurement(
                observation.pose(), observation.timeStamp(), observation.stdDevs());

            Logger.recordOutput(observation.camName()+"/stdDevX", observation.stdDevs().get(0));
            Logger.recordOutput(observation.camName()+"/stdDevY", observation.stdDevs().get(1));
            Logger.recordOutput(observation.camName()+"/stdDevTheta", observation.stdDevs().get(2));
            // Logger.recordOutput(observation.camName()+"/TransformFromOdometry", odometry.getPoseMeters().minus(observation.pose()));
        }

        poseEstimator.update(robotRotation, getModulePositions());
        odometry.update(robotRotation, getModulePositions());

        field.setRobotPose(getPoseEstimate());

        // Logger.recordOutput("Drive/Odometry/FieldCurrentChassisSpeeds", ChassisSpeeds.fromRobotRelativeSpeeds(getRobotChassisSpeeds(), robotRotation));

        /* Updating Controllers */
        headingController.updateHeadingController();
        autoAlignController.updateAlignmentControllers();
        GoalPoseChooser.updateSideStuff();

        ///////////////////// SETTING DESIRED SPEEDS FROM DRIVE STATE \\\\\\\\\\\\\\\\\\
        ChassisSpeeds teleopSpeeds = teleopController.computeChassiSpeeds(
            getPoseEstimate().getRotation(), getRobotChassisSpeeds(), false);
        switch (driveState) {
            case TELEOP:
                desiredSpeeds = teleopSpeeds;
                break;
            case TELEOP_SNIPER:
                desiredSpeeds = teleopController.computeChassiSpeeds(
                    getPoseEstimate().getRotation(), getRobotChassisSpeeds(), true);
                break;
            case POV_SNIPER:
                desiredSpeeds = teleopController.computeSniperPOVChassisSpeeds(getPoseEstimate().getRotation());
                break;
            case PROCESSOR_HEADING_ALIGN:
                goalRotation = AllianceFlipUtil.apply(Rotation2d.fromDegrees(90.0));
                desiredSpeeds = new ChassisSpeeds(
                    teleopSpeeds.vxMetersPerSecond, teleopSpeeds.vyMetersPerSecond,
                    headingController.getSnapOutput( getPoseEstimate().getRotation() ));
                break;
            case INTAKE_HEADING_ALIGN:
                goalRotation = AllianceFlipUtil.apply(GoalPoseChooser.getIntakePose(getPoseEstimate()).getRotation());
                desiredSpeeds = new ChassisSpeeds(
                    teleopSpeeds.vxMetersPerSecond, teleopSpeeds.vyMetersPerSecond,
                    headingController.getSnapOutput( getPoseEstimate().getRotation() ));
                break;
            case REEF_HEADING_ALIGN:
                goalRotation = AllianceFlipUtil.apply(GoalPoseChooser.turnFromReefOrigin(getPoseEstimate()));
                desiredSpeeds = new ChassisSpeeds(
                    teleopSpeeds.vxMetersPerSecond, teleopSpeeds.vyMetersPerSecond,
                    headingController.getSnapOutput( getPoseEstimate().getRotation() ));
                break;
            case DRIVE_TO_CORAL:
                desiredSpeeds = autoAlignController.calculate(goalPose, getPoseEstimate());
                break;
            case DRIVE_TO_INTAKE:
                desiredSpeeds = autoAlignController.calculate(goalPose, getPoseEstimate());
                break;
            case DRIVE_TO_BARGE:
                ChassisSpeeds autoAlignSpeeds = autoAlignController.calculate(goalPose, getPoseEstimate());;
                desiredSpeeds = new ChassisSpeeds(
                    autoAlignSpeeds.vxMetersPerSecond,
                    teleopSpeeds.vyMetersPerSecond,
                    autoAlignSpeeds.omegaRadiansPerSecond
                );
                break;
            case DRIVE_TO_ALGAE:
                // desiredSpeeds = autoAlignController.calculate(goalPose, getPoseEstimate());
                ChassisSpeeds algaeAlignSpeeds = autoAlignController.calculate(goalPose, getPoseEstimate());
                double forwardJoy = (goalPose.getX() > AllianceFlipUtil.apply(FieldConstants.kReefCenter.getX()))
                ? -teleopSpeeds.vxMetersPerSecond: teleopSpeeds.vxMetersPerSecond;
                if(AllianceFlipUtil.shouldFlip()) forwardJoy *= -1;
                desiredSpeeds = new ChassisSpeeds(
                    /* Flips speed to preserve field relative. Not best solution, but probably good enough? */
                    forwardJoy,
                    algaeAlignSpeeds.vyMetersPerSecond,
                    algaeAlignSpeeds.omegaRadiansPerSecond
                );
                break;
            case AUTON:
                desiredSpeeds = ppDesiredSpeeds;
                break;
            case UP:
                desiredSpeeds = new ChassisSpeeds(0.5, 0.0, teleopSpeeds.omegaRadiansPerSecond);
                break;
            case DOWN:
                desiredSpeeds = new ChassisSpeeds(-0.5, 0.0, teleopSpeeds.omegaRadiansPerSecond);
                break;
            case LEFT:
                desiredSpeeds = new ChassisSpeeds(0.0, -0.5, teleopSpeeds.omegaRadiansPerSecond);
                break;
            case RIGHT:
                desiredSpeeds = new ChassisSpeeds(0.0, 0.5, teleopSpeeds.omegaRadiansPerSecond);
                break;
            case DRIFT_TEST:
                desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(linearTestSpeedMPS.get(), 0.0, 
                        Math.toRadians(rotationDriftTestSpeedDeg.get())), robotRotation);
                break;
            case LINEAR_TEST:
                desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(linearTestSpeedMPS.get(), 0.0, 0.0), robotRotation);
                break;
            /* Set by characterization commands in the CHARACTERIZATION header. Wheel characterization is currently unimplemented */
            case SYSID_CHARACTERIZATION:
            case WHEEL_CHARACTERIZATION:
                /* If null, then PID isn't set, so characterization can set motors w/o interruption */
                desiredSpeeds = null;
            case STOP:
                for(int i = 0; i < modules.length; i++) {
                    modules[i].setDesiredState(
                        new SwerveModuleState(0.0, modules[i].getCurrentState().angle));
                }
                break;
            default:
                /* Defaults to Teleop control if no other cases are run*/
        }

        ///////////////////////// SETS DESIRED CHASSIS SPEED TO MODULES \\\\\\\\\\\\\\\\\\\\\\\\
        if (desiredSpeeds != null) runSwerve(desiredSpeeds);
    }

    ///////////////////////// STATE SETTING \\\\\\\\\\\\\\\\\\\\\\\\
    public Command setDriveStateCommand(DriveState state) {
        return Commands.runOnce(() -> setDriveState(state), this);
    }

    /* Set's state initially, and doesn't end till interruped by another drive command */
    public Command setDriveStateCommandContinued(DriveState state) {
        return new FunctionalCommand(
            () -> setDriveState(state), () -> {}, 
            (interrupted) -> {}, () -> false, this);
    }

    /* Sets the drive state used in periodic(), and handles init condtions like resetting PID controllers */
    public void setDriveState(DriveState state) {
        driveState = state;
        switch(driveState) {
            case PROCESSOR_HEADING_ALIGN:
                headingController.reset(getPoseEstimate().getRotation(), gyroInputs.yawVelocityPS);            
                break;
            case REEF_HEADING_ALIGN:
                headingController.reset(getPoseEstimate().getRotation(), gyroInputs.yawVelocityPS);
                break;
            case DRIVE_TO_CORAL:
                goalPose = GoalPoseChooser.getGoalPose(CHOOSER_STRATEGY.kReefHexagonal, getPoseEstimate());
                autoAlignController.setConstraintType(ConstraintType.LINEAR, getPoseEstimate(), goalPose);
                autoAlignController.reset(
                    getPoseEstimate(),
                    ChassisSpeeds.fromRobotRelativeSpeeds(
                        getRobotChassisSpeeds(), getPoseEstimate().getRotation()),
                    goalPose);
                break;
            case DRIVE_TO_INTAKE:
                goalPose = GoalPoseChooser.getGoalPose(CHOOSER_STRATEGY.kIntake, getPoseEstimate());
                autoAlignController.setConstraintType(ConstraintType.LINEAR, getPoseEstimate(), goalPose);
                autoAlignController.reset(
                    getPoseEstimate(), 
                    ChassisSpeeds.fromRobotRelativeSpeeds(
                        getRobotChassisSpeeds(), getPoseEstimate().getRotation()),
                    goalPose);
                break;
            case DRIVE_TO_ALGAE:
                goalPose = GoalPoseChooser.getGoalPose(CHOOSER_STRATEGY.kReefHexagonal, getPoseEstimate());
                autoAlignController.setConstraintType(ConstraintType.AXIS, getPoseEstimate(), goalPose);
                autoAlignController.reset(
                    getPoseEstimate(),
                    ChassisSpeeds.fromRobotRelativeSpeeds(
                        getRobotChassisSpeeds(), getPoseEstimate().getRotation()),
                    goalPose);
                break;
            case DRIVE_TO_BARGE:
                goalPose = GoalPoseChooser.getGoalPose(CHOOSER_STRATEGY.kNet, getPoseEstimate());
                autoAlignController.setConstraintType(ConstraintType.AXIS, getPoseEstimate(), goalPose);
                autoAlignController.reset(
                    getPoseEstimate(), 
                    ChassisSpeeds.fromRobotRelativeSpeeds(
                        getRobotChassisSpeeds(), getPoseEstimate().getRotation()),
                    goalPose);
                break;
            default:
        }
    }

    ////////////// CHASSIS SPEED TO MODULES \\\\\\\\\\\\\\\\
    /* Sets the desired swerve module states to the robot */
    public void runSwerve(ChassisSpeeds speeds) {
        desiredSpeeds = SwerveUtils.discretize(speeds, driftRate.get());

        /* Logs all the possible drive states, great for debugging */
        SwerveUtils.logPossibleDriveStates(kDoExtraLogging, desiredSpeeds, getModuleStates(), previousSetpoint, robotRotation);

        SwerveModuleState[] unOptimizedSetpointStates = new SwerveModuleState[4];
        SwerveModuleState[] setpointStates = kKinematics.toSwerveModuleStates(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, kMaxLinearSpeedMPS);

        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

        previousSetpoint = setpointGenerator.generateSetpoint(
            previousSetpoint, desiredSpeeds, kDriveConstraints, 0.02);

        /* Only for logging purposes */
        SwerveModuleState[] moduleTorques = SwerveUtils.zeroStates();

        // Logger.recordOutput("Drive/Odometry/generatedFieldSpeeds", ChassisSpeeds.fromRobotRelativeSpeeds(previousSetpoint.robotRelativeSpeeds(), robotRotation));

        for (int i = 0; i < 4; i++) {
            if(useGenerator) {
                /* Logs the drive feedforward stuff */
                SwerveUtils.logDriveFeedforward(previousSetpoint.feedforwards(), i);

                setpointStates[i] = new SwerveModuleState(
                    previousSetpoint.moduleStates()[i].speedMetersPerSecond,
                    /* setpointAngle = currentAngle if the speed is less than 0.01 */
                    SwerveUtils.removeAzimuthJitter(
                        previousSetpoint.moduleStates()[i], modules[i].getCurrentState()));

                unOptimizedSetpointStates[i] = SwerveUtils.copyState(setpointStates[i]);

                setpointStates[i].optimize(modules[i].getCurrentState().angle);

                /* Feedforward cases based on driveState */
                /* 0 unless in auto or auto-align */
                double driveAmps = calculateDriveFeedforward(
                    previousSetpoint, modules[i].getCurrentState(), unOptimizedSetpointStates[i], setpointStates[i], i);
                
                /* 
                 * Multiplies by cos(angleError) to stop the drive from going in the wrong direction
                 * when azimuth angle changes
                 */
                setpointStates[i].cosineScale(modules[i].getCurrentState().angle);

                double directionOfVelChange = Math.signum(setpointStates[i].speedMetersPerSecond - prevStates[i].speedMetersPerSecond);
                Logger.recordOutput("Drive/Module/Feedforward/"+i+"/dir", directionOfVelChange);
                if(driveState.equals(DriveState.AUTON)) {
                    driveAmps = Math.abs(driveAmps) * Math.signum(directionOfVelChange);
                }

                optimizedSetpointStates[i] = modules[i].setDesiredStateWithAmpFF(setpointStates[i], driveAmps);

                moduleTorques[i] = new SwerveModuleState((driveAmps * kMaxLinearSpeedMPS / 80), optimizedSetpointStates[i].angle);
            } else {
                setpointStates[i] = new SwerveModuleState(
                    setpointStates[i].speedMetersPerSecond,
                    SwerveUtils.removeAzimuthJitter(
                        setpointStates[i], modules[i].getCurrentState()));

                setpointStates[i].optimize(modules[i].getCurrentState().angle);
                setpointStates[i].cosineScale(modules[i].getCurrentState().angle);
                optimizedSetpointStates[i] = modules[i].setDesiredState(setpointStates[i]);
            }
        }

        prevStates = optimizedSetpointStates;
        
        Logger.recordOutput("Drive/Swerve/Setpoints", unOptimizedSetpointStates);
        Logger.recordOutput("Drive/Swerve/SetpointsOptimized", optimizedSetpointStates);
        Logger.recordOutput("Drive/Swerve/SetpointsChassisSpeeds", kKinematics.toChassisSpeeds(optimizedSetpointStates));
        Logger.recordOutput("Drive/Odometry/FieldSetpointChassisSpeed", ChassisSpeeds.fromRobotRelativeSpeeds(
            kKinematics.toChassisSpeeds(optimizedSetpointStates), robotRotation));
        Logger.recordOutput("Drive/Swerve/ModuleTorqueFF", moduleTorques);
    }

    /* Calculates DriveFeedforward based off state */
    public double calculateDriveFeedforward(SwerveSetpoint setpoint, SwerveModuleState currentState, SwerveModuleState unoptimizedState, SwerveModuleState optimizedState, int i) {
        switch(driveState) {
            case AUTON:
                /* No need to optimize for Choreo, as it handles it under the hood */
                return SwerveUtils.convertChoreoNewtonsToAmps(currentState, pathPlanningFF, i);
            case DRIVE_TO_CORAL: 
            case DRIVE_TO_INTAKE:
                return SwerveUtils.optimizeTorque(unoptimizedState, optimizedState, setpoint.feedforwards().torqueCurrentsAmps()[i], i);
            default:
                return 0.0;
        }
    }

    ////////////// LOCALIZATION(MAINLY RESETING LOGIC) \\\\\\\\\\\\\\\\
    public void resetGyro() {
        /* Robot is usually facing the other way(relative to field) when doing cycles on red side, so gyro is reset to 180 */
        robotRotation = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red) ? 
            Rotation2d.fromDegrees(180.0) : Rotation2d.fromDegrees(0.0);
        gyro.resetGyro(robotRotation);
        setPose(new Pose2d(new Translation2d(), robotRotation));
    }

    public void setPose(Pose2d pose) {
        setPoses(pose, pose);
    }

    public void setPoses(Pose2d estimatorPose, Pose2d odometryPose) {
        robotRotation = estimatorPose.getRotation();
        gyro.resetGyro(robotRotation);
        // Safe to pass in odometry poses because of the syncing
        // between gyro and pose estimator in reset gyro function
        poseEstimator.resetPosition(getRobotRotation(), getModulePositions(), estimatorPose);
        odometry.resetPosition(getRobotRotation(), getModulePositions(), odometryPose);
    }

    public void resetModulesEncoders() {
        for (int i = 0; i < 4; i++) modules[i].resetAzimuthEncoder();
    }

    //////////////////////// CHARACTERIZATION \\\\\\\\\\\\\\\\\\\\\\\\\\
    /* LINEAR CHARACTERIZATION: The x-y movement of the drivetrain(basically drive motor feedforward) */
    public Command characterizeLinearMotion() {
        return setDriveStateCommand(DriveState.SYSID_CHARACTERIZATION).andThen(
            SysIDCharacterization.runDriveSysIDTests( (voltage) -> {
                runLinearCharcterization(voltage);
        }, this));
    }

    public Command waitUnitllAutoAlignFinishes() {
        return new WaitUntilCommand(()-> autoAlignDelay.calculate(autoAlignController.atGoal()));
    }

    // public Command waitUnitllIntakeAutoAlignFinishes() {
    //     return new WaitUntilCommand(()-> autoAlignController.atGoal() || 
    //         autoAlignTimeout.calculate(autoAlignController.atPositionTimeout()));
    // }

    public BooleanSupplier waitUnitllAutoAlignFinishesSupplier() {
        return ()-> autoAlignController.atGoal();
    }

    /* Runs the robot forward at a voltage */
    public void runLinearCharcterization(double volts) {
        setDriveState(DriveState.SYSID_CHARACTERIZATION);
        for(int i = 0; i < 4; i++) modules[i].runCharacterization(volts);
    }

    public void setForwardAmperagesForAllModules(double amps) {
        for(int i = 0; i < 4; i++) {
            modules[i].setDesiredVelocity(null);
            modules[i].setDriveAmperage(amps);
            modules[i].setDesiredRotation(Rotation2d.fromDegrees(0.0));
        }
    }

    /* 
     * ANGULAR CHARACTERIZATION: The angular movement of the drivetrain(basically used to get drivebase MOI)
     * https://choreo.autos/usage/estimating-moi/
     */
    public Command characterizeAngularMotion() {
        return setDriveStateCommand(DriveState.SYSID_CHARACTERIZATION).andThen(
            SysIDCharacterization.runDriveSysIDTests( (voltage) -> runAngularCharacterization(voltage), this));
    }

    /* Runs the rotate's robot at a voltage */
    public void runAngularCharacterization(double volts) {
        setDriveState(DriveState.SYSID_CHARACTERIZATION);
        modules[0].runCharacterization( volts, Rotation2d.fromDegrees(-45.0));
        modules[1].runCharacterization(-volts, Rotation2d.fromDegrees( 45.0));
        modules[2].runCharacterization( volts, Rotation2d.fromDegrees( 45.0));
        modules[3].runCharacterization(-volts, Rotation2d.fromDegrees(-45.0));
    }

    public void runMOICharacterization(double amps) {
        setDriveState(DriveState.SYSID_CHARACTERIZATION);
        modules[0].setDriveAmperage(amps);
        modules[1].setDriveAmperage(-amps);
        modules[2].setDriveAmperage(amps);
        modules[3].setDriveAmperage(-amps);

        modules[0].setDesiredRotation(Rotation2d.fromDegrees(-45.0));
        modules[1].setDesiredRotation(Rotation2d.fromDegrees(45.0));
        modules[2].setDesiredRotation(Rotation2d.fromDegrees(45.0));
        modules[3].setDesiredRotation(Rotation2d.fromDegrees(-45.0));

        Logger.recordOutput("Drive/MOI/RadiansVelocity", modules[0].getInputs().driveVelocityMPS / DriveConstants.kDrivebaseRadiusMeters);
        Logger.recordOutput("Drive/MOI/DriveTorqueNM", 
            (SwerveUtils.getTorqueOfKrakenDriveMotor(modules[0].getInputs().driveTorqueCurrentAmps) * kDriveMotorGearing / kWheelRadiusMeters) * kDrivebaseRadiusMeters);
    }

    ///////////////////////// GETTERS \\\\\\\\\\\\\\\\\\\\\\\\
    @AutoLogOutput(key = "Drive/Swerve/MeasuredStates")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) states[i] = modules[i].getCurrentState();
        return states;
    }

    @AutoLogOutput(key = "Drive/Swerve/ModulePositions")
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) positions[i] = modules[i].getCurrentPosition();
        return positions;
    }

    @AutoLogOutput(key = "Drive/Odometry/PoseEstimate")
    public Pose2d getPoseEstimate() {
        return (RobotBase.isReal()) ? poseEstimator.getEstimatedPosition() : getOdometryPose();
    }

    @AutoLogOutput(key = "Drive/Odometry/OdometryPose")
    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    @AutoLogOutput(key = "Drive/Odometry/RobotRotation")
    public Rotation2d getRobotRotation() {
        return robotRotation;
    }

    @AutoLogOutput(key = "Drive/Odometry/RobotChassisSpeeds")
    public ChassisSpeeds getRobotChassisSpeeds() {
        return kKinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Drive/Odometry/DesiredChassisSpeeds")
    public ChassisSpeeds getDesiredChassisSpeeds() {
        return desiredSpeeds;
    }

    @AutoLogOutput(key = "Drive/Tolerance/HeadingController")
    public boolean inHeadingTolerance() {
        /* Accounts for angle wrapping issues with rotation 2D error */
        return GeomUtil.getSmallestChangeInRotation(robotRotation, goalRotation).getDegrees() 
            < HeadingController.toleranceDegrees.get();
    }

    @AutoLogOutput(key = "Drive/Odometry/DistanceFromReef")
    public double distanceFromReefCenter(){
        return getPoseEstimate().getTranslation().getDistance(AllianceFlipUtil.apply(kReefCenter).getTranslation());
    }

    public void acceptJoystickInputs(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, DoubleSupplier povSupplierDegrees) {
        teleopController.acceptJoystickInputs(xSupplier, ySupplier, thetaSupplier, povSupplierDegrees);
    }

    public boolean atGoal(){
        return autoAlignController.atGoal() && driveState == DriveState.DRIVE_TO_CORAL;
    }

    public boolean notAtGoal(){
        return !autoAlignController.atGoal();
    }

    public boolean getDriveToPoseTolerance() {
        return autoAlignController.atGoal();
    }
}