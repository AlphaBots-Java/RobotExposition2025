package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.opencv.core.Mat;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AnguladorSubsystem{
    
    
    public SparkMax cansparkMaxAng = new SparkMax(57, MotorType.kBrushless);
    public CANcoder canCoder = new CANcoder(46, "rio");
    PIDController pidControllerAng = new PIDController(1.6, 0, 0.1);
    Encoder caEncoder;
    ArmFeedforward feedforward = new ArmFeedforward(0.3, 0.3, 12.24, 0.02);
    // ArmFeedforward feedforward = new ArmFeedforward(0, 0.4, 0, 0);
    private double AnguladorSetPoint;

    // private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1.5);

    private final TrapezoidProfile.Constraints m_constraints =
        new TrapezoidProfile.Constraints(0.6, 0.4);
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private double angleOffsetDeg = 24;

    private final Timer m_timer = new Timer();
    private PS5Controller cont = new PS5Controller(0);



    SparkMaxConfig config = new SparkMaxConfig();
    
    
    
    
    public void AnguladorSubsystem(){
        config 
            .idleMode(IdleMode.kBrake);
            config.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
            config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1.5, 0.5, 1);

        cansparkMaxAng.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public void AngSetPoint(){
        m_goal = new TrapezoidProfile.State(AnguladorSetPoint, 0);
        var profile = new TrapezoidProfile(m_constraints);

        m_setpoint = profile.calculate(0.02, m_setpoint, m_goal);

        SmartDashboard.putNumber("anguladorSetPoint", m_setpoint.position );
        cansparkMaxAng.set(-feedforward.calculate((m_setpoint.position + 30/360) *2 * Math.PI, -m_setpoint.velocity)/ 18 + pidControllerAng.calculate(canCoder.getPosition().getValueAsDouble(), m_setpoint.position));
        SmartDashboard.putNumber("feedforward", feedforward.calculate(m_setpoint.position *2 * Math.PI, -m_setpoint.velocity)/ 18);

    }



    public void AngUpdateSetPoint(double Angsetpoint){
        this.AnguladorSetPoint = Angsetpoint;
        m_timer.restart();
    }



}
