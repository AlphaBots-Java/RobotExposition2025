package frc.robot.Commands;

import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.Subsystems.AnguladorSubsystem;


public class AnguladorController {
    
    PS5Controller controller = new PS5Controller(0);
 
    AnguladorSubsystem angulador = new AnguladorSubsystem();
    public void ControllAngulador(){
       
      angulador.AngSetPoint();

    }

    public void UpdateSetPoint(){
        // 1 setpoint = 7mm;;;
        if(controller.getTriangleButtonReleased()){
            angulador.AngUpdateSetPoint(-0.51);
        }
        if(controller.getCircleButtonReleased()){
            angulador.AngUpdateSetPoint(-0.05);
        }
        if(controller.getSquareButtonReleased()){
            angulador.AngUpdateSetPoint(-0.05);
        }
        if(controller.getCrossButtonReleased()){
            angulador.AngUpdateSetPoint(0);
        }
    }

}
