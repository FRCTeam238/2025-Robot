// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.SignalLogger;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CoralMechanismState;
import frc.robot.autonomous.AutonomousModesReader;
import frc.robot.autonomous.DataFileAutonomousModeDataSource;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // @NotLogged
  public Pivot pivot;
  // @NotLogged
  public Drivetrain drivetrain;
  public Elevator elevator;
  public Controls controls;
  public CoralIntake coralIntake;
  // @NotLogged
  public Wrist wrist;
  // @NotLogged`
  public AlgaeIntake algae;

  @NotLogged
  PowerDistribution pd;

  public static CoralMechanismState coralState = CoralMechanismState.Stow;


  private AutonomousModesReader autoReader;
  private List<String> autoNames;
  private SendableChooser<String> autoChooser;
  private String lastSelectedAuto;


  public Robot() {
    CanBridge.runTCP();
    SignalLogger.start();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    URCL.start();
    Epilogue.bind(this);

    pd = new PowerDistribution(1, ModuleType.kRev);
    
    elevator = Elevator.getInstance();
    pivot = Pivot.getInstance();
    drivetrain = Drivetrain.getInstance();
    wrist = Wrist.getInstance();
    coralIntake = CoralIntake.getInstance();
    algae = AlgaeIntake.getInstance();
    controls = Controls.getInstance();
    
  }
  
  @Override
  public void robotInit() {
    pd.setSwitchableChannel(true);
    autoReader = new AutonomousModesReader(new DataFileAutonomousModeDataSource(Filesystem.getDeployDirectory() + "/amode238.txt"));

    autoChooser = new SendableChooser<String>();

    autoNames = autoReader.getAutoNames();

    for (String name : autoNames) {
        autoChooser.setDefaultOption(name, name);
    }
    SmartDashboard.putData(autoChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (lastSelectedAuto != autoChooser.getSelected()) {
      m_autonomousCommand = autoReader.getAutonomousMode(autoChooser.getSelected());
    }

    lastSelectedAuto = autoChooser.getSelected();
  }

  @Override
  public void disabledInit() {
    pivot.stop();
    elevator.stop();
    wrist.stop();
    algae.stopAll();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    algae.setPosition(new MotionProfile.State(0));
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  public CoralMechanismState getCoralState() {
    return coralState;
  }
}
