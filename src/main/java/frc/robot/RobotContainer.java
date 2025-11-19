// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Commands.AnguladorController;
import frc.robot.Commands.ElevatorController;
import frc.robot.Commands.LimeLightCommand;
import frc.robot.Commands.SugadorController;
import frc.robot.Commands.SubidorController;
import frc.robot.Commands.SwerveCommand;
import frc.robot.Subsystems.LimeLightSubsystem;
import frc.robot.Subsystems.SugadorSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final Joystick controller = new Joystick(0);
  private final PS5Controller buttonController = new PS5Controller(0);
  private final LimeLightSubsystem lls = new LimeLightSubsystem();

  private final AnguladorController angulador = new AnguladorController();
  private final SugadorController sugador = new SugadorController();
  private final ElevatorController elevator = new ElevatorController();
  private final SubidorController subidor = new SubidorController();
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
  public void PrintThings(){
    angulador.printAngler();
    SmartDashboard.putNumber("DistanceAPRTAG", lls.DistanceToTarget() -4);
  }

  public void PegadorDeCoralTeleop(){
    elevator.ControllElevator();
    elevator.UpdateSetPoint();
    angulador.ControllAngulador();
    angulador.UpdateSetPoint();
    sugador.ControllVellocity();
    sugador.UpdateVellocity();
    subidor.UpdateSetPoint();
    subidor.ControllAngler();
    configureBindings();
  }

  public void PegadorDeCoralAuto(){
    elevator.ControllElevator();
    elevator.UpdateSetPoint();
    angulador.ControllAngulador();
    angulador.UpdateSetPoint();
    sugador.ControllVellocity();
    subidor.UpdateSetPoint();
    configureBindings();
  }

  private void configureBindings() {
    if(buttonController.getOptionsButtonPressed()){
      swerve.zeroHeading();
    }
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Example Auto");
  }
  
}
