package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Localization extends SubsystemBase {

    public Pose2d estimatedPose = new Pose2d();

    private static Localization _instance;
    public static Localization getInstance() {
        if(_instance == null) _instance = new Localization();
        return _instance;
    }
}