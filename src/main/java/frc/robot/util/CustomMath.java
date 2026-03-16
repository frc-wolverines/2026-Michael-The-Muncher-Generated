package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;

public class CustomMath {
    public static Rotation2d normalizeRotationHalfAndHalf(Rotation2d rotation) {
        double degrees = rotation.getDegrees();
        degrees = (degrees + 180) % 360 - 180;
        return Rotation2d.fromDegrees(degrees);
    }

    public static Pose2d makePoseAllianceRelative(Pose2d pose) {
        Alliance alliance = DriverStation.getAlliance().get();
        if(alliance == Alliance.Red) {
            pose = new Pose2d(
                FieldConstants.fieldWidth - pose.getX(), 
                FieldConstants.fieldLength - pose.getY(), 
                normalizeRotationHalfAndHalf(pose.getRotation().minus(Rotation2d.k180deg)));
        }
        return pose;
    }
}