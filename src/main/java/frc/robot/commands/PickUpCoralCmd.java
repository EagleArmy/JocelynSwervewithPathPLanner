package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;

/**
 * A simple command that grabs a hatch with the {@link AMPSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class PickUpCoralCmd extends Command {
  // The subsystem the command runs on
  private final CoralIntakeSubsystem m_coralintakeSubsystem;

  public PickUpCoralCmd(CoralIntakeSubsystem subsystem) {
    m_coralintakeSubsystem = subsystem;
    addRequirements(m_coralintakeSubsystem);
  }


  @Override
  public void initialize() { }

  @Override
  public void execute() {
    if (!(m_coralintakeSubsystem.getSensorRange() < 300)) {
        m_coralintakeSubsystem.intakecoral();
    }
    else {
        m_coralintakeSubsystem.stop();
      System.out.println("CORAL SENSOR TRIGGERED!!!!");
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_coralintakeSubsystem.stop(); 
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}