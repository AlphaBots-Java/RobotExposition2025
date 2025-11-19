package frc.robot.Subsystems; 
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;


public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
    SwerveModulePosition[] saas;
        
    private final Pigeon2 pigeon = new Pigeon2(5, "SwerveCAN");
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(0),
                                                                        new SwerveModulePosition[]{
                                                                            frontLeft.GetModulePosition(),
                                                                            frontRight.GetModulePosition(), 
                                                                            backLeft.GetModulePosition(), 
                                                                            backRight.GetModulePosition(),}
                                                                            );
    

    public SwerveSubsystem() {

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();


        RobotConfig config;
        try{
        config = RobotConfig.fromGUISettings();
        
    
        AutoBuilder.configure(this::getPose,
                              this::resetOdometry,
                              () -> DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                                                                              frontRight.getState(),
                                                                              backLeft.getState(), 
                                                                              backRight.getState()), 
                              (speeds, feedforward) -> driveRobotRelative(speeds),  
                              new PPHolonomicDriveController( // PathPlanner`s lib for controlling drivetrains
                              new PIDConstants(5.0, 0.0, 0.0), 
                              new PIDConstants(5.0, 0.0, 0.0)),
                              config ,
                              () -> {
                                var alliance = DriverStation.getAlliance();
                                if (alliance.isPresent()) {
                                    return alliance.get() == DriverStation.Alliance.Red;
                                }
                                return false;
                              },
                              this);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void zeroHeading() {
        pigeon.reset();
    }

    //Correcao da orientacao do robo (-90)
    public double getHeading() {
        return Math.IEEEremainder(pigeon.getYaw().getValueAsDouble() - 90,360);
        // return -90;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }


    public Pose2d getPose() {
       return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
       odometer.resetPosition(getRotation2d(), new SwerveModulePosition[]{
                                                frontLeft.GetModulePosition(),
                                                frontRight.GetModulePosition(), 
                                                backLeft.GetModulePosition(), 
                                                backRight.GetModulePosition()}, pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), new SwerveModulePosition[]{frontLeft.GetModulePosition(),
                                                                    frontRight.GetModulePosition(),
                                                                    backLeft.GetModulePosition(), 
                                                                    backRight.GetModulePosition()});


        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        SmartDashboard.putNumber("angle0", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("angle1", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("angle2", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("angle3", backRight.getAbsoluteEncoderRad());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0], frontLeft.getRotation2d());
        frontRight.setDesiredState(desiredStates[1], frontRight.getRotation2d());
        backLeft.setDesiredState(desiredStates[2], backLeft.getRotation2d());
        backRight.setDesiredState(desiredStates[3], backRight.getRotation2d());
    }

    public void driveRobotRelative(ChassisSpeeds Mspeeds){
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(Mspeeds);
        this.setModuleStates(moduleStates);
    }
}