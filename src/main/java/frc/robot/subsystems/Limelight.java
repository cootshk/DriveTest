package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.math.controller.PIDController;
/* Limelight position relative to middle of front of robot at floor level:
 * 
 * 
 * 
 * 
 */

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    SwerveDrive m_robotDrive;
    PIDController rotationController = new PIDController(0.6, 0, 0);

    public Limelight(SwerveDrive m_robotDrive){
        this.m_robotDrive=m_robotDrive;
    }

    //printing values for debugging
    public void getValue(){
        DriverStation.Alliance color;
        color = DriverStation.getAlliance();
        if(color == DriverStation.Alliance.Blue){
            System.out.println(limelight.getEntry("botpose_wpired").getDoubleArray(new double[6])[0] + "," + limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[1] + "," + limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[2] + "," + limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[3] + "," + limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[4] + "," + limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[5]);
        }
    }

    //check whether or not limelight sees the apriltag
    public boolean hasPose() {
        // REMINDER: Add condition
        //CHECK IF CAN SEE LIMELIGHT
        DriverStation.Alliance color;
        color = DriverStation.getAlliance();
        if(color == DriverStation.Alliance.Blue){
            double[] values = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            for(int i = 0; i < values.length-1; i++){
                if(values[i] != 0){
                    return true;
                }
            }
            return false;
        } else {
            double[] values = limelight.getEntry("botpose_wpired").getDoubleArray(new double[6]);
            for(int i = 0; i < values.length-1; i++){
                if(values[i] != 0){
                    return true;
                }
            }
            return false;
        }
    }

    //get the position of the robot as Pose2d from limelight
    public TimestampPose2d getTimestampedPose() {
        DriverStation.Alliance color;
        color = DriverStation.getAlliance();
        double[] data = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
        if(color == DriverStation.Alliance.Red){
            data = limelight.getEntry("botpose_wpired").getDoubleArray(new double[7]);
        }
        double x;
        double y;
        double yaw;

        if(color == DriverStation.Alliance.Blue){
            x = data[0];
            y = data[1];
            yaw = data[5];
        } else {
            x = data[0];
            y = data[1];
            yaw = data[5];
        }

        TimestampPose2d pose = new TimestampPose2d(new Pose2d(x,y, Rotation2d.fromDegrees(yaw)), Timer.getFPGATimestamp() - (data[6]/1000.0));
        // GET CURRENT POSE FROM LIMELIGHT
        return pose;
    }

    // Wrapper for estimated Pose2d and pose timestamp
    public class TimestampPose2d {
        private double timestamp;
        private Pose2d pose;
        
        public TimestampPose2d(Pose2d pose, double timestamp) {
            // REMINDER: Use Timer.getFGPAtimestamp or something like that or get from limelight networktables
            this.timestamp = timestamp;
            this.pose = pose;
        }

        public Pose2d getPose2d() {
            return pose;
        }

        public double getTimestamp() {
            return timestamp;
        }
    }
}
