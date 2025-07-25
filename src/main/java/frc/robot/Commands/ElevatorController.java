package frc.robot.Commands;

import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorController {
     PS5Controller controller = new PS5Controller(0);
    ElevatorSubsystem elevator = new ElevatorSubsystem();
    public void ControllElevator(){
        elevator.SetPoint();
       

    }

    public void UpdateSetPoint(){
        // 1 setpoint = 7mm
        if(controller.getTriangleButtonPressed()){
            elevator.UpdateSetPoint(0);
        }
        if(controller.getSquareButtonPressed()){
            elevator.UpdateSetPoint(70.8);
        }
        if(controller.getCircleButtonPressed()){
            elevator.UpdateSetPoint(13);
        }
        if(controller.getCrossButtonPressed()){
            elevator.UpdateSetPoint(0);
        }
    }

}
