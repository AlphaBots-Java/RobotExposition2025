package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PS5Controller;

public class ElevatorSubsystem {
    
    public SparkMax cansparkMax = new SparkMax(56, MotorType.kBrushless);
    PIDController pidController = new PIDController(0.5, 0, 0);
    Encoder caEncoder;
    private double elevatorSetPoint = 0;


    SparkMaxConfig config = new SparkMaxConfig();
    
    
    
    
    public void ElevatorSubsystem(){
        config 
            .inverted(true)
            .idleMode(IdleMode.kBrake);
            config.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
            config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.01, 0.0, 0);

        cansparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void SetPoint(){
        cansparkMax.set(pidController.calculate(cansparkMax.getEncoder().getPosition(), elevatorSetPoint ) * 0.05);
        
    }
    public void UpdateSetPoint(double setpoint){
        this.elevatorSetPoint = setpoint;
    }


}
