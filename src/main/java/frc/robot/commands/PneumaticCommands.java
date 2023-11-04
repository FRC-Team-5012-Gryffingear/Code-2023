package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ripadaluz;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class PneumaticCommands extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final ripadaluz pneums;




  /**
   * Creates a new PneumaticCommands.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PneumaticCommands(ripadaluz subsystem) {



     pneums = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pneums.outlet(true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pneums.outlet(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
    
