package frc.robot.Commands;

import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.Subsystems.SugadorSubsystem;

public class SugadorController {
     PS5Controller controller = new PS5Controller(0);
 
    SugadorSubsystem sugador = new SugadorSubsystem();
    public void ControllVellocity(){
       
      sugador.SugSetVellocity();

    }

    public void UpdateVellocity(){
        // 1 setpoint = 7mm;;;
        if(controller.getR2Button()){
            sugador.SugUpdateSetVellocity(-0.45);
        }
        if (controller.getR2ButtonReleased()) {
            sugador.SugUpdateSetVellocity(0.05);
        }

        if(controller.getL2Button()){
            sugador.SugUpdateSetVellocity(0.4);
        }
        if (controller.getL2ButtonReleased()) {
            sugador.SugUpdateSetVellocity(0.05);
        }
        
    }
}
