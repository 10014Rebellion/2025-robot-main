package frc.robot.subsystems.auton;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.drive.controllers.GoalPoseChooser.SIDE;
import frc.robot.util.math.AllianceFlipUtil;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AutonCommands extends SubsystemBase {
    public static enum AutoState {
        PreScoreCoral,
        ScoreCoral,
        PreIntakeCoral,
        IntakeCoral
    }

    private SendableChooser<Command> autoChooser;

    private Drive robotDrive;

    private AutoState autoState = AutoState.PreScoreCoral;

    private Trigger preScoreCoral;
    private Trigger scoreCoral;
    private Trigger prIntakeCoral;
    private Trigger IntakeCoral;

    private Trigger IR1;
    private Trigger IR2;
    private Trigger IR3;

    private Trigger elevInIntakeWristExitRange;
    private Trigger wristInIntakeEntryRange;
    private Trigger wristInIntakeExitRange;
    private Trigger wristInScoringRange;
    
    private Trigger inAutoAlignRange;
    private Trigger inDrivingScoringTolerance;
    private Trigger inIntakeStartTolerance;
    private Trigger inIntakeTolerance;

    private Pose2d pose;

    public AutonCommands(Drive robotDrive) {
        // store subsystems
        this.robotDrive = robotDrive;

        autoChooser = new SendableChooser<>();

        autoChooser.setDefaultOption("Stationary", backUpAuton());
        tryToAddPathToChooser("String", nextScoreCoralPath("A1-Barge"));

        preScoreCoral = new Trigger(() ->  true);
        scoreCoral = new Trigger(() ->  true);
        prIntakeCoral = new Trigger(() ->  true);
        IntakeCoral = new Trigger(() ->  true);
    
        IR1 = new Trigger(() ->  true);
        IR2 = new Trigger(() ->  true);
        IR3 = new Trigger(() ->  true);
    
        elevInIntakeWristExitRange = new Trigger(() ->  true);
        wristInIntakeEntryRange = new Trigger(() ->  true);
        wristInIntakeExitRange = new Trigger(() ->  true);
        wristInScoringRange = new Trigger(() ->  true);
        
        inAutoAlignRange = new Trigger(() ->  true);
        inDrivingScoringTolerance = new Trigger(() ->  true);
        inIntakeStartTolerance = new Trigger(() ->  true);
        inIntakeTolerance = new Trigger(() ->  true);
    }

    ///////////////// PATH CHAINING LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public void tryToAddPathToChooser(String pathName, Command... commands) {
        for(Command path : commands) {
            tryToAddPathToChooser(pathName, new Runnable() {
                @Override
                public void run() {
                    autoChooser.addOption(pathName, path);
                }
            });
        }
    }  
    
    /* Stops magic auton errors from occuring due to FMS or some BS I cook up */
    public void tryToAddPathToChooser(String pathName, Runnable pathAdding) {
        try {
            pathAdding.run();
        } catch(Exception e) {
            autoChooser.addOption("Failed: "+pathName, backUpAuton());
        }
    }

    public SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }

    ///////////////// PATH CHAINING LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public PathPlannerAuto firstPath(String name, Rotation2d startingRotation, BooleanSupplier conditionSupplier, Command nextCommand, Command nextAuto) {
        PathPlannerAuto firstAuto = new PathPlannerAuto(followFirstChoreoPath(name, startingRotation));
        firstAuto.condition(conditionSupplier).onTrue(nextCommand.andThen(Commands.runOnce(() -> nextAutoChecker(nextAuto).schedule())));
        return firstAuto;
    }

    public PathPlannerAuto nextPath(String name, BooleanSupplier conditionSupplier, Command nextCommand, Command nextAuto) {
        PathPlannerAuto auto = new PathPlannerAuto(followChoreoPath(name));
        auto.condition(conditionSupplier).onTrue(nextCommand.andThen(Commands.runOnce(() -> nextAutoChecker(nextAuto).schedule())));
        return auto;
    }

    public PathPlannerAuto nextPath(String name, BooleanSupplier conditionSupplier, Command nextCommand, Command nextAuto, PPHolonomicDriveController PID) {
        PathPlannerAuto auto = new PathPlannerAuto(followChoreoPath(name, PID));
        auto.condition(conditionSupplier).onTrue(nextCommand.andThen(Commands.runOnce(() -> nextAutoChecker(nextAuto).schedule())));
        return auto;
    }

    public Command nextAutoChecker(Command auto) {
        return (auto == null) ? robotDrive.setDriveStateCommand(Drive.DriveState.STOP) : auto;
    }

    public Command backUpAuton() {
        return new InstantCommand();
    }

    public PathPlannerAuto nextScoreCoralPath(String name) {
        PathPlannerAuto auto = new PathPlannerAuto(autoPlaceholder());

        auto.activePath(name)
            .onTrue(followChoreoPath(name))
            .onTrue(elevatorToPreScoreCommand())
            .onTrue(wristToPreScoreCommand())
            .onTrue(intakePivotToPreScoreCommand())
            .onTrue(intakeToPreScoreCommand())
            .onTrue(indexerToPreScoreCommand());

        auto.condition(inAutoAlignRange)
            .onTrue(robotDrive.setDriveStateCommandContinued(DriveState.DRIVE_TO_CORAL));

        auto.condition(inDrivingScoringTolerance)
            .onTrue(wristToScoreCommand());

        auto.condition(inDrivingScoringTolerance.and(wristInScoringRange))
            .onTrue(clawEjectCommand());

        auto.condition(IR3.negate())
            .onTrue(clawHoldCommand())
            .onTrue(Commands.run(() -> auto.cancel()));

        return auto;
    }

    public PathPlannerAuto nextIntakeCoralPath(String name) {
        PathPlannerAuto auto = new PathPlannerAuto(autoPlaceholder());

        auto.activePath(name)
            .onTrue(followChoreoPath(name));

        auto.condition(inIntakeStartTolerance)
            .onTrue(elevatorToPreIntakeCommand())
            .onTrue(wristToPreIntakeCommand())
            .onTrue(intakePivotCoralCommand())
            .onTrue(
                new SequentialCommandGroup(
                    intakeCoralCommand(),
                    Commands.waitSeconds(1),
                    new RepeatCommand(
                        new SequentialCommandGroup(
                            setIntakeVoltCommand(12),
                            Commands.waitSeconds(0.1),
                            setIntakeVoltCommand(-12),
                            Commands.waitSeconds(0.1)
                        )
                    ).withTimeout(0.5)
                )
            )
            .onTrue(
                new SequentialCommandGroup(
                    intakeCoralCommand(),
                    Commands.waitSeconds(1),
                    new RepeatCommand(
                        new SequentialCommandGroup(
                            setIntakeVoltCommand(12),
                            Commands.waitSeconds(0.1),
                            setIntakeVoltCommand(-12),
                            Commands.waitSeconds(0.1)
                        )
                    ).withTimeout(0.5)
                )
            );

        auto.condition(IR1)
            .onTrue(new SequentialCommandGroup(
                Commands.waitSeconds(1),
                    new RepeatCommand(
                        new SequentialCommandGroup(
                            setIntakeVoltCommand(12),
                            Commands.waitSeconds(0.1),
                            setIntakeVoltCommand(-12),
                            Commands.waitSeconds(0.1)
                        )
                    ).withTimeout(0.5)
                )
            )
            .onTrue(new SequentialCommandGroup(
                Commands.waitSeconds(1),
                    new RepeatCommand(
                        new SequentialCommandGroup(
                            setIntakeVoltCommand(12),
                            Commands.waitSeconds(0.1),
                            setIntakeVoltCommand(-12),
                            Commands.waitSeconds(0.1)
                        )
                    ).withTimeout(0.5)
                )
            );

        auto.condition(IR2.and(wristInIntakeEntryRange))
            .onTrue(clawIntakeCommand())
            .onTrue(elevatorToIntakeCommand());

        auto.condition(IR3)
            .onTrue(clawHoldCommand())
            .onTrue(elevatorToPreScoreCommand());

        auto.condition(IR3.and(elevInIntakeWristExitRange))
            .onTrue(Commands.run(() -> auto.cancel()));

        return auto;
    }

    ///////////////// SUPERSTRUCTURE COMMANDS AND DATA \\\\\\\\\\\\\\\\\\\\\
    public Command clawHoldCommand() {
        return new InstantCommand();
    }

    public Command clawEjectCommand() {
        return new InstantCommand();
    }

    public Command clawIntakeCommand() {
        return new InstantCommand();
    }
    
    public Command elevatorToPreScoreCommand() {
        return new InstantCommand();
    }

    public Command wristToPreScoreCommand() {
        return new InstantCommand();
    }

    public Command intakeToPreScoreCommand() {
        return new InstantCommand();
    }

    public Command intakePivotToPreScoreCommand() {
        return new InstantCommand();
    }

    public Command indexerToPreScoreCommand() {
        return new InstantCommand();
    }

    public Command clawToPreScoreCommand() {
        return new InstantCommand();
    }

    public Command wristToScoreCommand() {
        return new PrintCommand("Score Coral");
    }

    public Command elevatorToPreIntakeCommand() {
        return new InstantCommand();
    }

    public Command wristToPreIntakeCommand() {
        return new InstantCommand();
    }

    public Command intakeCoralCommand() {
        return new PrintCommand("Intake Coral");
    }

    public Command setIntakeVoltCommand(double volts) {
        return new PrintCommand("Intake Coral Volts");
    }

    public Command intakePivotCoralCommand() {
        return new PrintCommand("Intake Coral");
    }

    public Command indexCoralCommand() {
        return new PrintCommand("Index Coral");
    }

    public Command setIndexVoltCommand(double volts) {
        return new PrintCommand("Intake Coral Volts");
    }

    public Command elevatorToIntakeCommand() {
        return new PrintCommand("Elevator To Intake");
    }

    public Command scoreAlgaeCommand() {
        return new PrintCommand("Score Algae");
    }

    public Command intakeAlgaeCommand() {
        return new PrintCommand("Intake Algae");
    }

    public BooleanSupplier getIntakeIR1HasPiece() {
        return () -> false;
    }

    public BooleanSupplier getIntakeIR2HasPiece() {
        return () -> false;
    }
    public BooleanSupplier getClawIR3HasPiece() {
        return () -> false;
    }

    public BooleanSupplier getIntakePivotAtGoal() {
        return () -> false;
    }

    public BooleanSupplier getElevatorAtGoal() {
        return () -> false;
    }

    public BooleanSupplier getWristAtGoal() {
        return () -> false;
    }    

    ///////////////// PATH CREATION LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public Command followFirstChoreoPath(String pathName, Rotation2d startingRotation) {
        PathPlannerPath path = getTraj(pathName).get();
        double totalTimeSeconds = path.getIdealTrajectory(Drive.robotConfig).get().getTotalTimeSeconds();

        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                robotDrive.setDriveState(DriveState.AUTON);
                robotDrive.setPose(AllianceFlipUtil.apply(new Pose2d(path.getPathPoses().get(0).getTranslation(), startingRotation)));
            }), 
            robotDrive.customFollowPathComamnd(path).withTimeout(totalTimeSeconds), 
            robotDrive.setDriveStateCommand(DriveState.STOP));
    }

    public Command followChoreoPath(String pathName) {
        PathPlannerPath path = getTraj(pathName).get();
        path.getIdealTrajectory(Drive.robotConfig);
        double totalTimeSeconds = path.getIdealTrajectory(Drive.robotConfig).get().getTotalTimeSeconds();
        return 
            robotDrive.setDriveStateCommand(DriveState.AUTON).andThen(
                robotDrive.customFollowPathComamnd(path).withTimeout(totalTimeSeconds), 
                robotDrive.setDriveStateCommand(DriveState.STOP));
    }

    public Command followChoreoPath(String pathName, PPHolonomicDriveController PID) {
        PathPlannerPath path = getTraj(pathName).get();
        path.getIdealTrajectory(Drive.robotConfig);
        double totalTimeSeconds = path.getIdealTrajectory(Drive.robotConfig).get().getTotalTimeSeconds();
        return 
            robotDrive.setDriveStateCommand(DriveState.AUTON).andThen(
                robotDrive.customFollowPathComamnd(path, PID).withTimeout(totalTimeSeconds), 
                robotDrive.setDriveStateCommand(DriveState.STOP));
    }

    public Optional<PathPlannerPath> getTraj(String pathName) {
        try {
            return Optional.of(PathPlannerPath.fromChoreoTrajectory(pathName));
        } catch(Exception e) {
            e.printStackTrace();
            return Optional.empty();
        }
    }

    public SIDE getSide(String name){
        String n = name.substring(6, 7);
        return (n.equals("L")) ? SIDE.LEFT : SIDE.RIGHT;
    }

    public FunctionalCommand autoPlaceholder() {
        return new FunctionalCommand(() -> {}, () -> {}, (interrupted) -> {}, () -> false, this);
    }
}