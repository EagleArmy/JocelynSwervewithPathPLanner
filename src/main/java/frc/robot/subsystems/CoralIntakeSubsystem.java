// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.PickupConstants;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.CoralLauncherConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * The claw subsystem is a simple system with a motor for opening and closing. If using stronger
 * motors, you should probably use a sensor so that the motors don't stall.
 */
public class CoralIntakeSubsystem extends SubsystemBase {
  private final TalonFX AlgaeLeftMotor = new TalonFX(AlgaeConstants.AlgaeLeftMotorID);
  private final TalonFX AlgaeRightMotor = new TalonFX(AlgaeConstants.AlgaeRightMotorID);
  private final TalonFX ElevatorMotor = new TalonFX(IndexerConstants.IndexerMotorID);
  private final TalonFX CoralIntakeLeftMotor = new TalonFX( CoralIntakeConstants.CoralIntakeLeftMotorID);
  private final TalonFX CoralIntakeRightMotor = new TalonFX( CoralIntakeConstants.CoralIntakeRighttMotorID);
  private final TalonFX CoralLauncherLeftMotor = new TalonFX( CoralLauncherConstants.CoralLauncherLeftMotorID);
  private final TalonFX CoralLauncherRightMotor = new TalonFX( CoralLauncherConstants.CoralLauncherRightMotorID);
  private final TimeOfFlight algaesensor = new TimeOfFlight(12);
  private final TimeOfFlight coralsensor = new TimeOfFlight(13);
  private boolean test = true;

  
  public CoralIntakeSubsystem() {
    // Let's name everything on the LiveWindow
    addChild("AlgaeLeftMotor", AlgaeLeftMotor);
    addChild("AlgaeRightMotor", AlgaeRightMotor);
    addChild("ElevatorMotor", ElevatorMotor);
    addChild("CoralIntakeLeftMotor", CoralIntakeLeftMotor);
    addChild("CoralIntakeRightMotor", CoralIntakeRightMotor);
    addChild("CoralLauncherLeftMotor", CoralLauncherLeftMotor);
    addChild("CoralLauncherRightMotor", CoralLauncherRightMotor);
    addChild("coralsensor", coralsensor);

    var elevatorConfiguration = new TalonFXConfiguration();
    elevatorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ElevatorMotor.getConfigurator().apply( elevatorConfiguration );

    var algaeConfiguration = new TalonFXConfiguration();
    algaeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    AlgaeLeftMotor.getConfigurator().apply( algaeConfiguration );
    AlgaeRightMotor.getConfigurator().apply( algaeConfiguration );

    coralsensor.setRangingMode( TimeOfFlight.RangingMode.Short, 24);
  }

  public void log() { }


  public void intakecoral() {
    if (test) {
        CoralIntakeLeftMotor.set(0.5);
        CoralIntakeRightMotor.set(-0.5);
    }
    else {
      stop();
    }
  }

  public void stop() {
    CoralIntakeLeftMotor.set(0.0);
    CoralIntakeRightMotor.set(0.0);
  }

  public void stopAuto() {
    CoralIntakeLeftMotor.set(0.0);
    CoralIntakeRightMotor.set(0.0);
  }

  public void intakecoralAuto() {
    System.out.println("Value: " + getSensorRange());
    if (!(getSensorRange() < 120)) {
      intakecoral();
    }
    else {
      stop();
    }
  }

  public double getSensorRange() {
    return coralsensor.getRange();
  }

  public boolean isEmpty() 
  {
    return (coralsensor.getRange() > 120);
  }

  public boolean isFull() 
  {
    return (coralsensor.getRange() < 120);
  }

  public double getCoralSensorLimit() {
    return SensorConstants.CoralSensorLimit;
  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();
  }
}