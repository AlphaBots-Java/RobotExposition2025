package frc.robot.Commands;

import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorController {
     PS5Controller controller = new PS5Controller(1);
    ElevatorSubsystem elevator = new ElevatorSubsystem();
    public void ControllElevator(){
        elevator.SetPoint();
       

    }

    public void UpdateSetPoint(){
        // 1 setpoint = 7mm
        if(controller.getR1ButtonPressed()){
            //pegar peca
            elevator.UpdateSetPoint(0);
        }
        if(controller.getR2ButtonPressed()){
            //L3
            elevator.UpdateSetPoint(70.8);
        }
        if(controller.getL2ButtonPressed()){
            //L2
            elevator.UpdateSetPoint(13);
        }
        if(controller.getL1ButtonPressed()){
            //L1
            elevator.UpdateSetPoint(0);
        }
    }

    public void ElevatorL2(){
        elevator.UpdateSetPoint(13);
    }
    public void ElevatorL1(){
        elevator.UpdateSetPoint(0);
    }
    public void ElevatorL3(){
        elevator.UpdateSetPoint(70);
    }

}
