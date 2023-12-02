package frc.robot.commands;

import frc.robot.subsystems.ArcadeDriveSSTest;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** An example command that uses an example subsystem. */
public class ArcadeDriveComTest extends CommandBase {
    private final DoubleSupplier forwardTrig1;
    private final DoubleSupplier backwardTrig1;
    private final DoubleSupplier turning;
    private final BooleanSupplier shoot;

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArcadeDriveSSTest m_subsystem;

  public ArcadeDriveComTest(ArcadeDriveSSTest subsystem, DoubleSupplier ForwardTrigger, DoubleSupplier BackwardTrigger, DoubleSupplier Turning, BooleanSupplier Shoot) {
    m_subsystem = subsystem;
    forwardTrig1 = ForwardTrigger;
    backwardTrig1 = BackwardTrigger;
    turning = Turning;
    shoot = Shoot;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.moveAndTurn(forwardTrig1.getAsDouble() - backwardTrig1.getAsDouble(), turning.getAsDouble());
    m_subsystem.turnAndShoot(shoot.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

