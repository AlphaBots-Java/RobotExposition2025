package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Radians;

import java.util.MissingFormatWidthException;

import javax.sound.midi.Soundbank;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;



public class SwerveModule {
    
    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
  
    private final PIDController turningPidController;
    
    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId, "SwerveCAN");

        driveMotor = new TalonFX(driveMotorId, "SwerveCAN");
        turningMotor = new TalonFX(turningMotorId, "SwerveCAN");

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        turningPidController = new PIDController(0.5, 0.05, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

     
        // Orchestra orchestra = new Orchestra();

        // orchestra.addInstrument(driveMotor);
        // orchestra.loadMusic('./MarioSound.mp3');
           
    }

    

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        return turningMotor.getPosition().getValueAsDouble() * ModuleConstants.kTurningEncoderRot2Rad;
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    public double getTurningVelocity() {
        return turningMotor.getVelocity().getValueAsDouble() * ModuleConstants.kTurningEncoderRPM2RadPerSec;
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition().getValue().in(Radians);
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        turningMotor.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(getAbsoluteEncoderRad());
    }

    public void setDesiredState(SwerveModuleState state, Rotation2d rotation2d) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        
        rotation2d = new Rotation2d(getAbsoluteEncoderRad());

        state.optimize(rotation2d);
        
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians() - absoluteEncoderOffsetRad));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public SwerveModulePosition GetModulePosition(){
        return new SwerveModulePosition(
        this.getDrivePosition()
        ,this.getRotation2d()
        );
    }
}