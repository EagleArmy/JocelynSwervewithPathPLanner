package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/**
 * A simple command that grabs a hatch with the {@link ShooterSubsystem} and {@link MaxVelocitySubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class DropAlgaeCmd extends Command {
  // The subsystem the command runs on
  private final AlgaeSubsystem m_algaeSubsystem;

  public DropAlgaeCmd(AlgaeSubsystem subsystem) {
    m_algaeSubsystem = subsystem;
    addRequirements(m_algaeSubsystem);
  }

  
  @Override
  public void initialize() { }

  @Override
  public void execute() {
    m_algaeSubsystem.dropalgae();
  }

  @Override
  public void end(boolean interrupted) {
    m_algaeSubsystem.stop(); 
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
}