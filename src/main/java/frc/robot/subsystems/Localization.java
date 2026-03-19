package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AlertContainer;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
public class Localization extends SubsystemBase {
    public Alert llDisconnect;

    Localization() {
        llDisconnect = AlertContainer.getInstance().register(new Alert("Limelight is disconnected", AlertType.kError));
    }

    @Override
    public void periodic() {
        llDisconnect.set(LimelightHelpers.getHeartbeat("limelight") == 0);
        Drivetrain drivetrain = Drivetrain.getInstance();
        SwerveDriveState state = drivetrain.getStateCopy();
        LimelightHelpers.SetRobotOrientation(
            "limelight", 
            state.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        PoseEstimate pose = getVisionMT2Pose();
        if(pose.tagCount > 0) drivetrain.addVisionMeasurement(pose.pose, pose.timestampSeconds);
    }

    public LimelightHelpers.PoseEstimate getVisionMT2Pose() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    }

    private static Localization _instance;
    public static Localization getInstance() {
        if(_instance == null) _instance = new Localization();
        return _instance;
    }
}