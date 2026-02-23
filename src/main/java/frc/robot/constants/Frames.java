package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;

public class Frames {
    public record IntakeState(Rotation2d rotation, Voltage voltage) {}
    public record RobotState(Pose2d position, IntakeState intakeState, Rotation2d turretRotation) {}
}