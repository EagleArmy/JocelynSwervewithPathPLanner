package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * A simple command that grabs a hatch with the {@link PositionSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class AlgaeDownPositionCmd extends Command {
  // The subsystem the command runs on
  private ElevatorSubsystem m_elevatorSubsystem;

  public AlgaeDownPositionCmd(ElevatorSubsystem subsystem) {
    m_elevatorSubsystem = subsystem;
    addRequirements(m_elevatorSubsystem);
  }


  @Override
  public void initialize() { }
  

  @Override
  public void execute() {
    m_elevatorSubsystem.backward();
  }

  @Override
  public void end(boolean interrupted) { }

  @Override
  public boolean isFinished() {
    return false;
  }
  
}
