package frc.lib.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriftCorrection {
    
    /**
     * Drift correction PID used to correct our rotation in teleop
     * @param speeds Chassis speeds object from drive subsystem
     * @param pose Current pose from odometry
     * @return Updated Chassis Speeds
     */

    private static double lockAngle = 0;
    private static boolean locked = false;
    private static PIDController rotationCorrection = new PIDController(0.5, 0, 0);
    
    public static void configPID()
    {
        rotationCorrection.enableContinuousInput(0, 360);
    }

    public static ChassisSpeeds driftCorrection(ChassisSpeeds speeds, Pose2d pose)
    {
        SmartDashboard.putBoolean("Rotation Locked", locked);
        SmartDashboard.putNumber("Lock Angle", lockAngle);
        SmartDashboard.putNumber("Current Angle", pose.getRotation().getDegrees());

        SmartDashboard.putNumber("Rotation Natural Target", speeds.omegaRadiansPerSecond);

        if(speeds.omegaRadiansPerSecond == 0.0)
        {
            // Not trying to rotate, attempt to maintain angle
            if(locked)
            {
                // Angle already locked on
                speeds.omegaRadiansPerSecond = rotationCorrection.calculate(pose.getRotation().getDegrees());
                SmartDashboard.putNumber("Rotation Correction", speeds.omegaRadiansPerSecond);
                return speeds;
            }
            else
            {
                // No angle locked, acquire lock
                lockAngle = pose.getRotation().getDegrees() % 360;
                rotationCorrection.setSetpoint(lockAngle);
                locked = true;
                return speeds;
            }
        }
        else
        {
            locked = false;
            return speeds;
        }
    }

    /*public static ChassisSpeeds driftCorrection(ChassisSpeeds speeds, Pose2d pose) {
        PIDController driftCorrectionPID = new PIDController(0.07, 0.00, 0.004);
        double desiredHeading= 0;
        double pXY = 0;
        ChassisSpeeds newSpeeds = speeds;

        double xy = Math.abs(newSpeeds.vxMetersPerSecond) + Math.abs(newSpeeds.vyMetersPerSecond);

        if (Math.abs(newSpeeds.omegaRadiansPerSecond) > 0.0 || pXY <= 0)
            desiredHeading = pose.getRotation().getDegrees();
        else if (xy > 0)
            newSpeeds.omegaRadiansPerSecond += driftCorrectionPID.calculate(pose.getRotation().getDegrees(),
                    desiredHeading);
        
        pXY = xy;

        return newSpeeds;
    }*/

}
