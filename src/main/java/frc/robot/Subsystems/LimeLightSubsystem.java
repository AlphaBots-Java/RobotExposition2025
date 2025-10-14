package frc.robot.Subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class LimeLightSubsystem {
    public double maxSpeedX = 0.3;
    public double maxSpeedY = 0.3;

    public double limelightStrafeProportional(){    
        // double kP = .015;
        double kP = .02;

        double targetingAngularVelocity = LimelightHelpers.getTX("limelight-esq") * kP;
        SmartDashboard.putNumber("LLTX", targetingAngularVelocity);
        // limite de velocidade strafe
        if(targetingAngularVelocity > maxSpeedX){
            targetingAngularVelocity = maxSpeedX;
        }else if (targetingAngularVelocity < -maxSpeedX){
            targetingAngularVelocity = -maxSpeedX;
        }

        // targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    public double DistanceToTarget()
    {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-esq");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 7.0; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 6.5; 

        // distance from the target to the floor
        double goalHeightInches = 12.25; 

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        //calculate distance
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    public double LimelightRunProportional(){
        // double kp = 0.01;
        // double targetingVelocity = (this.DistanceToTarget() -3) * kp;

        // targetingVelocity *= -0.3;



        double kp = 0.0032;
        double targetingVelocity = (this.DistanceToTarget()) * kp;
        targetingVelocity *= -1;

        // limite de velocidade frente
        if(targetingVelocity > maxSpeedY){
            targetingVelocity = maxSpeedY;
        }else if (targetingVelocity < -maxSpeedY){
            targetingVelocity = -maxSpeedY;
        }

        return targetingVelocity;
    }


}
