package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class CustomMath {
    public static Rotation2d normalizeRotationHalfAndHalf(Rotation2d rotation) {
        double degrees = rotation.getDegrees();
        degrees = (degrees + 180) % 360 - 180;
        return Rotation2d.fromDegrees(degrees);
    }
}