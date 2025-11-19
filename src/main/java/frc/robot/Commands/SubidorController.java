package frc.robot.Commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.SubidorSubsystem;

public class SubidorController {
    PS5Controller controller = new PS5Controller(1);
    SubidorSubsystem subidor = new SubidorSubsystem();
    DigitalInput LimitSwitchAng = new DigitalInput(3);
    private double MaxAnglerPosition = -160;

    private double MaxClawPosition = 10;
    private double MinimalClawPosition = 2;
    private DigitalInput MaxClawLimitSwitch = new DigitalInput(6);
    private DigitalInput MinClawLimitSwitch = new DigitalInput(2);

    public void ControllAngler(){

    }

    public void UpdateSetPoint(){
        SmartDashboard.putBoolean("LS", LimitSwitchAng.get());
        SmartDashboard.putBoolean("Sensor2", MinClawLimitSwitch.get());
        if(controller.getPOV() == 180){
            subidor.SetForceAng(-0.8);
        }
        else if(controller.getPOV() == 0){
            if(!LimitSwitchAng.get()){
                subidor.SetForceAng(0.8);
            }else{
                subidor.SetForceAng(-0.2);
            }
        }else{
            subidor.SetForceAng(0);
        }

        if(controller.getPOV() == 270){
            // if(MinClawLimitSwitch.get()){
                subidor.SetForceAbr(0.8);
            // }else{
            //     subidor.SetForceAbr(-0.5);
            // }
            
        }
        else if(controller.getPOV() == 90){
            // if(MaxClawLimitSwitch.get()){
                subidor.SetForceAbr(-0.8);
            // }else{
            //     subidor.SetForceAbr(0.5);
            // }

        }else{
            subidor.SetForceAbr(0);
        }
    }


}