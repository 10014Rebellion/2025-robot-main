package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorGoTo extends Command {
    private final ElevatorSubsystem mElevatorSubsystem;
    double mSetPoint;

    public ElevatorGoTo(double setPoint, ElevatorSubsystem pElevatorSubsystem) {
        this.mElevatorSubsystem = pElevatorSubsystem;
        this.mSetPoint = MathUtil.clamp(setPoint, ElevatorConstants.kMinHeight, ElevatorConstants.kMaxHeight);
        
        addRequirements(mElevatorSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        mElevatorSubsystem.doPID(mSetPoint);
    }

    @Override
    public void end(boolean interrupted) {
        mElevatorSubsystem.stop();

    }

    @Override
    public boolean isFinished() {
        return mElevatorSubsystem.PIDFinished();
    }
}
