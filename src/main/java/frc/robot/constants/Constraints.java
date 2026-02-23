package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constraints {
    public static final double TURRET_AZIMUTH_GEAR_RATIO = 5 * 10;
    public static final double INTAKE_PIVOT_GEAR_RATIO = 5 * 5 * (60/24);

    public static final Rotation2d TURRET_MIN_ROTATION = Rotation2d.fromDegrees(0);
    public static final Rotation2d TURRET_MAX_ROTATION = Rotation2d.fromDegrees(0);
}