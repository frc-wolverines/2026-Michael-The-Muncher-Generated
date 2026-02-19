package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class Frames {
    public record IntakeState(Rotation2d rotation, double voltage) {}
}