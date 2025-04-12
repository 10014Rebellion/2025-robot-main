package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.Supplier;

public class DynamicCommand extends InstantCommand {
  private final Supplier<Command> mCommandSupplier;

  public DynamicCommand(Supplier<Command> pCommandSupplier) {
    this.mCommandSupplier = pCommandSupplier;
  }

  @Override
  public void initialize() {
    Command dynamicCommand = mCommandSupplier.get();
    CommandScheduler.getInstance().schedule(dynamicCommand);
  }
}
