// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

//Imported Constants
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.CoralLauncherConstants;
import frc.robot.Constants.AlgaeConstants;

//Imported Subsystems
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.CoralLauncherSubsystem;


//Imported Commands

import frc.robot.commands.AlgaeDownPositionCmd;
import frc.robot.commands.AlgaeUpPositionCmd;
import frc.robot.commands.LimelightOrientationCmd;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);                    // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);      // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */                                         
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final Telemetry logger = new Telemetry(MaxSpeed);

    //Robot's Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ElevatorSubsystem m_Position = new ElevatorSubsystem();
    private final LimelightSubsystem m_Limelight = new LimelightSubsystem();
    //private ShuffleboardTab m_Tab;

    //Driver's Controller
    private final CommandXboxController joystick = new CommandXboxController(0);
    // XboxController m_driverController1 = new XboxController(OIConstants.kDriverControllerPort1);
    // XboxController m_driverController2 = new XboxController(OIConstants.kDriverControllerPort2);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;



    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

    
    //     m_Tab = Shuffleboard.getTab("COMPETITION_FINAL");
    //     // Limelight set up.
    //     HttpCamera limelight = new HttpCamera("Limelight", "http://10.34.88.11:5800/");
    //     CameraServer.startAutomaticCapture(limelight);
    //     m_Tab.add("Limelight", limelight)
    //         .withWidget(BuiltInWidgets.kCameraStream)
    //         .withPosition(7, 0)
    //         .withSize(10, 10);
    //         // .withSize(7, 5);

    // CommandScheduler.getInstance()
    //     .onCommandInitialize(
    //         command ->
    //             Shuffleboard.addEventMarker(
    //                 "Command initialized", command.getName(), EventImportance.kNormal));
    // CommandScheduler.getInstance()
    //     .onCommandExecute(
    //         command ->
    //             Shuffleboard.addEventMarker(
    //                 "Command executed", command.getName(), EventImportance.kNormal));
    // CommandScheduler.getInstance()
    //     .onCommandFinish(
    //         command ->
    //             Shuffleboard.addEventMarker(
    //                 "Command finished", command.getName(), EventImportance.kNormal));
    // CommandScheduler.getInstance()
    //     .onCommandInterrupt(
    //         command ->
    //             Shuffleboard.addEventMarker(
    //                 "Command interrupted", command.getName(), EventImportance.kNormal));
    }




    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)              // Drive forward with negative Y (forward)
                 .withVelocityY(-joystick.getLeftX() * MaxSpeed)              // Drive left with negative X (left)
                 .withRotationalRate(-joystick.getRightX() * MaxAngularRate)  // Drive counterclockwise with negative X (left)
            )
        );

        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX())) ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        joystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.a().whileTrue(new AlgaeDownPositionCmd(m_Position));
        joystick.b().whileTrue(new AlgaeUpPositionCmd(m_Position));
        joystick.x().whileTrue(new LimelightOrientationCmd(m_Limelight, drivetrain));
    }





    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
