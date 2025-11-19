package frc.robot.Commands;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.AnguladorSubsystem;


public class AnguladorController {
    
    PS5Controller controller = new PS5Controller(1);
 
    AnguladorSubsystem angulador = new AnguladorSubsystem();
    public void ControllAngulador(){
       
      angulador.AngSetPoint();

    }

    public void printAngler(){
        SmartDashboard.putNumber("AnguladorAngle", angulador.getAnglerAbsEncoder());
    }

    public void UpdateSetPoint(){
        if(controller.getR1ButtonReleased()){
            //pegar peca
         angulador.AngUpdateSetPoint(-0.52);
        }
        if(controller.getR2ButtonReleased()){
            //L3
            angulador.AngUpdateSetPoint(-0.05);
        }
        if(controller.getL2ButtonReleased()){
            //L2
            angulador.AngUpdateSetPoint(-0.04);
        }
        if(controller.getL1ButtonReleased()){
            //L1
            angulador.AngUpdateSetPoint(0);
        }
    }

    public void AnguladorL2(){
        angulador.AngUpdateSetPoint(-0.04);
    }
    public void AnguladorL1(){
        angulador.AngUpdateSetPoint(0);
    }

}
