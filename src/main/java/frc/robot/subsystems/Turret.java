package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Configs;
import frc.robot.constants.Constraints;
import frc.robot.constants.Map;
import frc.robot.constants.Tunables;

public class Turret extends SubsystemBase {
    private final TalonFX turretAzimuth;
    private final PIDController azimuthController = new PIDController(Tunables.TURRET_AZIMUTH_PID_CONSTANTS.kP, Tunables.TURRET_AZIMUTH_PID_CONSTANTS.kI, Tunables.TURRET_AZIMUTH_PID_CONSTANTS.kD);

    public Turret() {
        turretAzimuth = new TalonFX(Map.TURRET_AZIMUTH);
        turretAzimuth.getConfigurator().apply(Configs.TURRET_AZIMUTH_CONFIGURATION);
        turretAzimuth.setPosition(0);

        azimuthController.enableContinuousInput(-360,360);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Azimuth Degrees", getRotation().getDegrees());
        SmartDashboard.putNumber("Turret/Azimuth Field Degrees", getFieldRelativeRotation().getDegrees());
        SmartDashboard.putData(this);
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromRotations((turretAzimuth.getPosition().getValueAsDouble() % 1.0));
    }

    private Command dutyCycle(Supplier<Double> dutyCycleSupplier) {
        return Commands.runEnd(() -> {
            turretAzimuth.setControl(new DutyCycleOut(dutyCycleSupplier.get()));
        }, () -> turretAzimuth.setControl(new NeutralOut()), this);
    }

    private Command turnToRelativeAngle(Rotation2d angle) {
        return Commands.runEnd(() -> {
            turretAzimuth.setControl(
                new DutyCycleOut(
                    azimuthController.calculate(getRotation().getRotations(), angle.getRotations())
                )
            );
        }, () -> turretAzimuth.setControl(new NeutralOut()), this);
    }

    public Command turnToFieldAngle(Rotation2d angle) {
        return Commands.runEnd(() -> {
            if(fieldAngleUnachievable(angle)) return;
            Drivetrain drivetrain = Drivetrain.getInstance();
            double setpoint = (drivetrain.getRotation3d().getZ() / (Math.PI * 2)) - angle.getRotations();
            turretAzimuth.setControl(
                new DutyCycleOut(
                    azimuthController.calculate(getRotation().getRotations(), setpoint)
                )
            );
        }, () -> turretAzimuth.setControl(new NeutralOut()), this);
    }

    public Command turnToLandmark(Translation2d landmark) {
        return Commands.runEnd(() -> {
            Drivetrain drivetrain = Drivetrain.getInstance();
            Pose2d robotPose = drivetrain.getStateCopy().Pose;
            Rotation2d angle = robotPose.getTranslation().minus(landmark).getAngle();
            if(fieldAngleUnachievable(angle)) return;
            double setpoint = (drivetrain.getRotation3d().getZ() / (Math.PI * 2)) - angle.getRotations();
            turretAzimuth.setControl(
                new DutyCycleOut(
                    azimuthController.calculate(getRotation().getRotations(), setpoint)
                )
            );
        }, () -> turretAzimuth.setControl(new NeutralOut()), this);
    }

    public Rotation2d getFieldRelativeRotation() {
        Drivetrain drivetrain = Drivetrain.getInstance();
        double rotations = (drivetrain.getRotation3d().getZ() / (Math.PI * 2)) + getRotation().getRotations();
        return Rotation2d.fromRotations(rotations);
    }

    public boolean fieldAngleUnachievable(Rotation2d angle) {
        Drivetrain drivetrain = Drivetrain.getInstance();
        double setpoint = (drivetrain.getRotation3d().getZ() / (Math.PI * 2)) - angle.getRotations();
        return setpoint > Constraints.TURRET_MAX_ROTATION.getRotations() || setpoint < Constraints.TURRET_MIN_ROTATION.getRotations();
    }

    private static Turret _instance;
    public static Turret getInstance() {
        if(_instance == null) _instance = new Turret();
        return _instance;
    }
}