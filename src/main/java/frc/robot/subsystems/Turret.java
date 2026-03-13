package frc.robot.subsystems;

import java.lang.constant.DirectMethodHandleDesc;
import java.time.OffsetDateTime;
import java.util.function.Supplier;

import javax.naming.ldap.LdapContext;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.constants.Configs;
import frc.robot.constants.Constraints;
import frc.robot.constants.Map;
import frc.robot.constants.Tunables;
import frc.robot.util.CustomMath;

public class Turret extends SubsystemBase {
    private final TalonFX turretAzimuth;
    private final PIDController azimuthController = new PIDController(Tunables.TURRET_AZIMUTH_PID_CONSTANTS.kP, Tunables.TURRET_AZIMUTH_PID_CONSTANTS.kI, Tunables.TURRET_AZIMUTH_PID_CONSTANTS.kD);
    private final DutyCycleEncoder azimuthEncoder;
    public Translation2d currentLandmark = Translation2d.kZero;

    public Turret() {
        turretAzimuth = new TalonFX(Map.TURRET_AZIMUTH);
        turretAzimuth.getConfigurator().apply(Configs.TURRET_AZIMUTH_CONFIGURATION);
        turretAzimuth.setPosition(0);
        azimuthEncoder = new DutyCycleEncoder(1);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Azimuth Degrees", getRotation().getDegrees());
        SmartDashboard.putNumber("Turret/Azimuth Degrees (rel)", turretAzimuth.getPosition().getValueAsDouble() * 360);
        SmartDashboard.putNumber("Turret/Azimuth Degrees (abs)", azimuthEncoder.get() * 360);
        SmartDashboard.putBoolean("Turret/Encoder Connected", azimuthEncoder.isConnected());
        SmartDashboard.putNumber("Turret/Azimuth Field Degrees", getFieldRelativeRotation().getDegrees());
        SmartDashboard.putData(this);
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromRotations(azimuthEncoder.get() - 0.5);
    }

    public Command turnToRelativeAngle(Rotation2d angle) {
        return Commands.runEnd(() -> {
            rotateTo(angle);
        }, () -> turretAzimuth.setControl(new NeutralOut()), this);
    }

    public Command turnToFieldAngle(Rotation2d angle) {
        return Commands.runEnd(() -> {
            Drivetrain drivetrain = Drivetrain.getInstance();
            Rotation2d relativeAngle = CustomMath.normalizeRotationHalfAndHalf(drivetrain.getState().Pose.getRotation().minus(angle).minus(Rotation2d.k180deg));
            if(angleUnacheivable(relativeAngle)) { turretAzimuth.setControl(new NeutralOut()); return; }
            rotateTo(relativeAngle);
        }, () -> turretAzimuth.setControl(new NeutralOut()), this);
    }

    public void rotateTo(Rotation2d angle) {
        double setpoint = Math.max(Constraints.TURRET_MIN_ROTATION.getRotations(), Math.min(angle.getRotations(), Constraints.TURRET_MAX_ROTATION.getRotations()));
        turretAzimuth.setControl(
                new DutyCycleOut(
                    azimuthController.calculate(getRotation().getRotations(), setpoint)
                )
            );
    }

    public Command turnToLandmark(Translation2d landmark, boolean accountForVelocity) {
        return Commands.runEnd(() -> {
            Translation2d newLandmark = landmark;
            Drivetrain drivetrain = Drivetrain.getInstance();
            SwerveDriveState state = drivetrain.getStateCopy();
            Pose2d robotPose = state.Pose;
            ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, robotPose.getRotation());
            if(accountForVelocity) {
                newLandmark = newLandmark.plus(new Translation2d(fieldSpeeds.vxMetersPerSecond * 0.1, fieldSpeeds.vyMetersPerSecond * 0.1));
            }
            Rotation2d translationAngle = newLandmark.minus(robotPose.getTranslation()).getAngle();
            Rotation2d relativeAngle = convertToRelative(translationAngle);
            relativeAngle = relativeAngle.plus(Rotation2d.fromRadians(state.Speeds.omegaRadiansPerSecond * 0.15));
            if(angleUnacheivable(relativeAngle)) { turretAzimuth.setControl(new NeutralOut()); return; }
            rotateTo(relativeAngle);
        }, () -> {
            turretAzimuth.setControl(new NeutralOut());
            currentLandmark = Translation2d.kZero;
        }, this).beforeStarting(() -> currentLandmark = landmark).withName("Pointing at Landmark");
    }

    public Rotation2d getFieldRelativeRotation() {
        Drivetrain drivetrain = Drivetrain.getInstance();
        return CustomMath.normalizeRotationHalfAndHalf(drivetrain.getState().Pose.getRotation().minus(getRotation()).minus(Rotation2d.k180deg));
    }

    public Rotation2d convertToRelative(Rotation2d rotation2d) {
        Drivetrain drivetrain = Drivetrain.getInstance();
        return CustomMath.normalizeRotationHalfAndHalf(drivetrain.getState().Pose.getRotation().minus(rotation2d).minus(Rotation2d.k180deg));
    }

    public boolean angleUnacheivable(Rotation2d rotation2d) {
        return rotation2d.getRotations() > Constraints.TURRET_MAX_ROTATION.getRotations() || rotation2d.getRotations() < Constraints.TURRET_MIN_ROTATION.getRotations();
    }

    private static Turret _instance;
    public static Turret getInstance() {
        if(_instance == null) _instance = new Turret();
        return _instance;
    }
}