package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public class Constraints {
    public static final double TURRET_AZIMUTH_GEAR_RATIO = 5 * 10;
    public static final double INTAKE_PIVOT_GEAR_RATIO = 5 * 5 * (60/24);
    public static final double INTAKE_ENCODER_OFFSET = 54;
    public static final double TURRET_ANTI_ZERO_OFFSET = 0.0;

    public static final Rotation2d TURRET_MIN_ROTATION = Rotation2d.fromDegrees(-135);
    public static final Rotation2d TURRET_MAX_ROTATION = Rotation2d.fromDegrees(135);

    public static final Distance ROBOT_WIDTH = Distance.ofBaseUnits(27, Inches);
    public static final Distance ROBOT_LENGTH = Distance.ofBaseUnits(27, Inches);
    public static final Distance BUMPER_THICKNESS = Distance.ofBaseUnits(5, Inches);

    public static final double KRAKEN_HIGH_TEMP = 70;
    public static final double KRAKEN_CRITICAL_TEMP = 80;
}   