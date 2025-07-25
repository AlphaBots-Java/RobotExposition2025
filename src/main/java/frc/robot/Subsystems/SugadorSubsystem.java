package frc.robot.Subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


public class SugadorSubsystem {
     

    public SparkMax cansparkMaxAng = new SparkMax(58, MotorType.kBrushless);


    private double SugadorSetVellocity = 0;
    

    public void SugSetVellocity(){
        cansparkMaxAng.set(SugadorSetVellocity);
    }

    public void SugUpdateSetVellocity(double SugsetVellocity){
        this.SugadorSetVellocity = SugsetVellocity;
    }


}
