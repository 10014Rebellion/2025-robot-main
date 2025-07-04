package frc.robot.util.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.function.Supplier;

public class DynamicCommand extends Command {
  private final Supplier<Command> mCommandSupplier;

  public DynamicCommand(Supplier<Command> pCommandSupplier) {
    this.mCommandSupplier = pCommandSupplier;
  }

  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(mCommandSupplier.get());
  }

  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().cancel(mCommandSupplier.get());
  }
}
