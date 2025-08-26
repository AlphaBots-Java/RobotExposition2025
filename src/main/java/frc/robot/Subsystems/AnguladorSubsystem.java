package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AnguladorSubsystem{
    
    
    public SparkMax cansparkMaxAng = new SparkMax(57, MotorType.kBrushless);
    private CANcoder canCoder = new CANcoder(46, "rio");
    PIDController pidControllerAng = new PIDController(1, 0, 0.1);
    Encoder caEncoder;
    private double AnguladorSetPoint = 0;

    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1.5);

    private final TrapezoidProfile.Constraints m_constraints =
        new TrapezoidProfile.Constraints(0.3, 0.15);
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private final Timer m_timer = new Timer();


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

        cansparkMaxAng.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void AngSetPoint(){
        m_goal = new TrapezoidProfile.State(AnguladorSetPoint, 0);
        var profile = new TrapezoidProfile(m_constraints);

        m_setpoint = profile.calculate(0.02, m_setpoint, m_goal);

        SmartDashboard.putNumber("posicao", canCoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("anguladorSpeed", m_setpoint.position);
        cansparkMaxAng.set(pidControllerAng.calculate(canCoder.getPosition().getValueAsDouble(), m_setpoint.position));
    }



    public void AngUpdateSetPoint(double Angsetpoint){
        this.AnguladorSetPoint = Angsetpoint;
        m_timer.restart();
    }



}
