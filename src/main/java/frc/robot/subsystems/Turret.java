package frc.robot.subsystems;

import java.lang.constant.DirectMethodHandleDesc;
import java.time.OffsetDateTime;
import java.util.function.Supplier;

import javax.naming.ldap.LdapContext;

 import com.ctre.phoenix6.Utils;
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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.constants.Configs;
import frc.robot.constants.Constraints;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Map;
import frc.robot.constants.Tunables;
import frc.robot.util.AlertContainer;
import frc.robot.util.CustomMath;

public class Turret extends SubsystemBase {
    private final TalonFX turretAzimuth;
    private final PIDController azimuthController = new PIDController(Tunables.TURRET_AZIMUTH_PID_CONSTANTS.kP, Tunables.TURRET_AZIMUTH_PID_CONSTANTS.kI, Tunables.TURRET_AZIMUTH_PID_CONSTANTS.kD);
    public Translation2d currentLandmark = Translation2d.kZero;
    public Rotation2d currentRelativeTarget = Rotation2d.kZero;
    public Alert highMotorTemp, criticalMotorTemp;

    public Turret() {
        turretAzimuth = new TalonFX(Map.TURRET_AZIMUTH);
        turretAzimuth.getConfigurator().apply(Configs.TURRET_AZIMUTH_CONFIGURATION);
        turretAzimuth.setPosition(0);

        setupAlerts();
        setDefaultCommand(turnToLandmark(FieldConstants.Hub.innerCenterPoint.toTranslation2d()));
    }

    public void setupAlerts() {
        highMotorTemp = AlertContainer.getInstance().register(new Alert("TURRET Motor has a high tempurature", AlertType.kWarning));
        criticalMotorTemp = AlertContainer.getInstance().register(new Alert("TURRET Motor has a critical tempurature", AlertType.kError));
    }

    public void updateAlerts() {
        boolean aboveHighTemp = turretAzimuth.getDeviceTemp().getValueAsDouble() > Constraints.KRAKEN_HIGH_TEMP;
        boolean aboveCriticalTemp = turretAzimuth.getDeviceTemp().getValueAsDouble() > Constraints.KRAKEN_CRITICAL_TEMP;
        highMotorTemp.set(aboveHighTemp && !aboveCriticalTemp);
        criticalMotorTemp.set(aboveCriticalTemp);
    }

    @Override
    public void periodic() {
        updateAlerts();
        SmartDashboard.putNumber("Turret/Azimuth Degrees", getRotation().getDegrees());
        SmartDashboard.putNumber("Turret/Azimuth Degrees (rel)", turretAzimuth.getPosition().getValueAsDouble() * 360);
        SmartDashboard.putNumber("Turret/Azimuth Field Degrees", getFieldRelativeRotation().getDegrees());
        SmartDashboard.putBoolean("Turret/Aligned", pointedTowardsTarget());
        SmartDashboard.putNumber("Turret/Target X", currentLandmark.getX());
        SmartDashboard.putNumber("Turret/Target Y", currentLandmark.getY());
        SmartDashboard.putData(this);
    }

    public Command lock() {
        return turnToRelativeAngle(Rotation2d.kZero);
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromRotations(turretAzimuth.getPosition().getValueAsDouble());
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

    public Command turnToLandmark(Translation2d landmark) {
        return Commands.runEnd(() -> {
            Translation2d newLandmark = CustomMath.makeTranslationAllianceRelative(landmark);
            Drivetrain drivetrain = Drivetrain.getInstance();
            SwerveDriveState state = drivetrain.getStateCopy();
            Pose2d robotPose = CustomMath.makePoseAllianceRelative(state.Pose);
            ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, CustomMath.makePoseAllianceRelative(robotPose).getRotation());
            double timeOfFlightCompensation = getTimeOfFlight() + (Utils.getCurrentTimeSeconds() - state.Timestamp);
            newLandmark = newLandmark.minus(new Translation2d(fieldSpeeds.vxMetersPerSecond * timeOfFlightCompensation, fieldSpeeds.vyMetersPerSecond * timeOfFlightCompensation));
            Rotation2d translationAngle = newLandmark.minus(CustomMath.makeTranslationAllianceRelative(robotPose.getTranslation())).getAngle();
            Rotation2d relativeAngle = convertToRelative(translationAngle);
            relativeAngle = relativeAngle.plus(Rotation2d.fromRadians(state.Speeds.omegaRadiansPerSecond * 0.15));
            if(angleUnacheivable(relativeAngle)) { turretAzimuth.setControl(new NeutralOut()); return; }
            currentRelativeTarget = relativeAngle;
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

    public boolean pointedTowardsTarget() {
        return Math.abs(getRotation().minus(currentRelativeTarget).getRotations()) < 0.2;
    }

    public double getTimeOfFlight() {
        // Using a quadratic regression based on distance to target to estimate time of flight
        Drivetrain drivetrain = Drivetrain.getInstance();
        Pose2d robotPose = CustomMath.makePoseAllianceRelative(drivetrain.getStateCopy().Pose);
        double distance = robotPose.getTranslation().getDistance(CustomMath.makeTranslationAllianceRelative(currentLandmark));
        return 1.55;
    }

    private static Turret _instance;
    public static Turret getInstance() {
        if(_instance == null) _instance = new Turret();
        return _instance;
    }
}