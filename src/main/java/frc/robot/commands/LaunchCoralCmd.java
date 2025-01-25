package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralLauncherSubsystem;

/**
 * A simple command that grabs a hatch with the {@link ShooterSubsystem} and {@link MaxVelocitySubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class LaunchCoralCmd extends Command {
  // The subsystem the command runs on
  private final CoralLauncherSubsystem m_corallauncherSubsystem;

  public LaunchCoralCmd(CoralLauncherSubsystem subsystem) {
    m_corallauncherSubsystem = subsystem;
    addRequirements(m_corallauncherSubsystem);
  }

  
  @Override
  public void initialize() { }

  @Override
  public void execute() {
    // m_maxvelocitySubsystem.testing();
    m_corallauncherSubsystem.launchcoral();
  }

  @Override
  public void end(boolean interrupted) {
    m_corallauncherSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
}