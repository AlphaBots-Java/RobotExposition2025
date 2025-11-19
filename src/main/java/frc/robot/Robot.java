// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.Console;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;


  final TalonFX joe = new TalonFX(21);
  
  public Robot() {
    m_robotContainer = new RobotContainer();


    m_Orchestra.addInstrument(joe);
    var status = m_Orchestra.loadMusic("./death.chrp");
    if (!status.isOK()) {
      System.out.println("Error Loading Music");
   }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.PrintThings();

    SmartDashboard.putBoolean("isPlaying", m_Orchestra.isPlaying());

  }

  @Override
  public void disabledInit() {
    m_Orchestra.stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    m_robotContainer.PegadorDeCoralAuto();
  }

  @Override
  public void autonomousExit() {}

  Orchestra m_Orchestra = new Orchestra();

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  new Thread(() -> {
            try {
                Thread.sleep(1000);
                m_Orchestra.play();
            } catch (Exception e) {
            }
        }).start();
  
}

  @Override
  public void teleopPeriodic() {
    m_robotContainer.PegadorDeCoralTeleop();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
