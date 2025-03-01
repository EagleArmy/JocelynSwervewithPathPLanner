// package frc.robot.commands;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.LimelightSubsystem;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import com.ctre.phoenix6.swerve.SwerveRequest;

 

// /**
//  * A simple command that grabs a hatch with the {@link PositionSubsystem}. Written explicitly for
//  * pedagogical purposes. Actual code should inline a command this simple with {@link
//  * edu.wpi.first.wpilibj2.command.InstantCommand}.
//  */
// public class LimelightOrientationCmd extends Command {
//   // The subsystem the command runs on
//   private LimelightSubsystem m_LimelightSubsystem;
//   private CommandSwerveDrivetrain m_Drivetrain;
//   private SwerveRequest.FieldCentric drive;

//   public LimelightOrientationCmd(LimelightSubsystem subsystem1, CommandSwerveDrivetrain subsystem2) {
//     m_LimelightSubsystem = subsystem1;
//     m_Drivetrain = subsystem2;
//     addRequirements(m_LimelightSubsystem, m_Drivetrain);
//   }


//   @Override
//   public void initialize() { }
  

//   @Override
//   public void execute() {
//     m_LimelightSubsystem.limelightOverride();

//         // Drivetrain will execute this command periodically
//         m_Drivetrain.applyRequest(() ->
//             drive.withVelocityX(m_LimelightSubsystem.getxSpeed())              
//                  .withVelocityY(m_LimelightSubsystem.getySpeed())             
//                  .withRotationalRate(m_LimelightSubsystem.getrot())  
//             );

//   }

//   @Override
//   public void end(boolean interrupted) { }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
  
// }
