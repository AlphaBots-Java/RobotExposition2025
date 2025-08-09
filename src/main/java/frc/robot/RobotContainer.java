// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.AnguladorController;
import frc.robot.Commands.ElevatorController;
import frc.robot.Commands.SugadorController;
import frc.robot.Commands.SwerveCommand;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final Joystick controller = new Joystick(0);
  private final PS5Controller buttonController = new PS5Controller(0);

  private final AnguladorController angulador = new AnguladorController();
  private final SugadorController sugador = new SugadorController();
  private final ElevatorController elevator = new ElevatorController();

  public RobotContainer() {
    Supplier<Double> axisZero = () -> this.controller.getRawAxis(0);
    Supplier<Double> axisOne = () -> this.controller.getRawAxis(1);
    Supplier<Double> axisTwo = () -> this.controller.getRawAxis(2);
    Supplier<Boolean> buttonSup = () -> this.buttonController.getPSButtonPressed();

    swerve.setDefaultCommand(new SwerveCommand(
      this.swerve,
      axisZero,
      axisOne,
      axisTwo,
      buttonSup
    ));
    
  }

  public void PegadorDeCoral(){
    elevator.ControllElevator();
    elevator.UpdateSetPoint();
    angulador.ControllAngulador();
    angulador.UpdateSetPoint();
    sugador.ControllVellocity();
    sugador.UpdateVellocity();
    configureBindings();
  }

  private void configureBindings() {
    if(buttonController.getOptionsButtonPressed()){
      swerve.zeroHeading();
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
