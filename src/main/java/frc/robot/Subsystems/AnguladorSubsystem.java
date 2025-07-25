package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;

public class AnguladorSubsystem {
    
    
    public SparkMax cansparkMaxAng = new SparkMax(57, MotorType.kBrushless);
    private CANcoder canCoder = new CANcoder(46, "rio");
    PIDController pidControllerAng = new PIDController(1, 0, 0.1);
    Encoder caEncoder;
    private double AnguladorSetPoint = 0;


    SparkMaxConfig config = new SparkMaxConfig();
    
    
    
    
    public void AnguladorSubsystem(){
        config 
            .inverted(true)
            .idleMode(IdleMode.kBrake);
            config.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
            config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1, 0.2, 0.1);

        cansparkMaxAng.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void AngSetPoint(){
        cansparkMaxAng.set(pidControllerAng.calculate(canCoder.getPosition().getValueAsDouble(), AnguladorSetPoint));
    }
    public void AngUpdateSetPoint(double Angsetpoint){
        this.AnguladorSetPoint = Angsetpoint;
    }



}
