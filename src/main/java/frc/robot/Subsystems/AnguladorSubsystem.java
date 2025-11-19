package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AnguladorSubsystem{
    
    
    SparkMax cansparkMaxAng = new SparkMax(57, MotorType.kBrushless);
    CANcoder canCoder = new CANcoder(46, "rio");
    PIDController pidControllerAng = new PIDController(1.6, 0, 0.1);
    ArmFeedforward feedforward = new ArmFeedforward(0.3, 0.3, 12.24, 0.02);
    // ArmFeedforward feedforward = new ArmFeedforward(0, 0.4, 0, 0);
    private double AnguladorSetPoint;

    private double kTrapezoidalProfilePeriod = 0.02;
    private double kTrapezoidalProfileMaxVelocity = 0.6;
    private double kTrapezoidalProfileMaxAcceleration = 0.4;


    // private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1.5);

    private final TrapezoidProfile.Constraints m_constraints =
        new TrapezoidProfile.Constraints(kTrapezoidalProfileMaxVelocity, kTrapezoidalProfileMaxAcceleration); //rotations/second and rotations/second²
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private double angleOffsetDeg = 24;

    private final Timer m_timer = new Timer();



    SparkMaxConfig config = new SparkMaxConfig();
    
    
    
    
    public AnguladorSubsystem(){
        config 
            .idleMode(IdleMode.kBrake);
            config.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
            config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        cansparkMaxAng.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public void AngSetPoint(){
        m_goal = new TrapezoidProfile.State(AnguladorSetPoint, 0);
        var profile = new TrapezoidProfile(m_constraints);

        m_setpoint = profile.calculate(kTrapezoidalProfilePeriod, m_setpoint, m_goal);

        SmartDashboard.putNumber("anguladorSetPoint", m_setpoint.position );

        var setPointDeg = m_setpoint.position * 360;
        var adjustedSetPointRadians = (setPointDeg + angleOffsetDeg / 180.0 ) * Math.PI;
        var setPointVelocityRadians = m_setpoint.velocity * 2 * Math.PI;

        cansparkMaxAng.set(-feedforward.calculate(adjustedSetPointRadians, -setPointVelocityRadians) + pidControllerAng.calculate(canCoder.getPosition().getValueAsDouble(), m_setpoint.position)); //both the encoder and trapezoidal are in rotations, so they match here
        SmartDashboard.putNumber("feedforward", feedforward.calculate(m_setpoint.position *2 * Math.PI, -m_setpoint.velocity * 2 * Math.PI));

    }



    public void AngUpdateSetPoint(double Angsetpoint){
        this.AnguladorSetPoint = Angsetpoint;
        m_timer.restart();
    }

    public double getAnglerAbsEncoder(){
        return canCoder.getPosition().getValueAsDouble();
    }

}
