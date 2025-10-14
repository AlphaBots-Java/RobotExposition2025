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
        // 1 setpoint = 7mm;
        if(controller.getL2Button()){
            sugador.SugUpdateSetVellocity(-0.45);
        }
        else if(controller.getR1Button()){
            sugador.SugUpdateSetVellocity(0.4);
        }
        else {
            sugador.SugUpdateSetVellocity(0.06);
        }
        
    }

    public void Sugar(){
        sugador.SugUpdateSetVellocity(0.45);
    }

    public void Soltar(){
        sugador.SugUpdateSetVellocity(-0.4);
    }
    public void Parar(){
        sugador.SugUpdateSetVellocity(0);
    }
}
