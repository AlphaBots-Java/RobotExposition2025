package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;

public class SubidorSubsystem {
    public SparkMax cansparkMaxAng = new SparkMax(55, MotorType.kBrushless);
    Encoder caEncoderAng;

    SparkMaxConfig configAng = new SparkMaxConfig();
    
    public SparkMax cansparkMaxAbr = new SparkMax(54, MotorType.kBrushed);
    Encoder caEncoderAbr;

     SparkMaxConfig configAbr = new SparkMaxConfig();
    
    
    
    public void SubidorSubsystemAng(){
        configAng
            .inverted(true)
            .idleMode(IdleMode.kBrake);
            configAng.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
            configAng.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        cansparkMaxAng.configure(configAng, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }



public void SubidorSubsystemAbr(){
    configAbr
        .inverted(true)
        .idleMode(IdleMode.kBrake);
        configAbr.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);
        configAbr.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    cansparkMaxAbr.configure(configAbr, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

public void SetForceAng(double force){
    cansparkMaxAng.set(force);
}

public void SetForceAbr(double force){
    this.cansparkMaxAbr.set(force);
}



}