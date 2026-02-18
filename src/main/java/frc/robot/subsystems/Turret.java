package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Dictionary;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Configs;
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
        SmartDashboard.putNumber("Turret/Azimuth Degrees", getTurretRotation().getDegrees());
        SmartDashboard.putData(this);
    }

    public Rotation2d getTurretRotation() {
        return Rotation2d.fromRotations((turretAzimuth.getPosition().getValueAsDouble() % 1.0));
    }

    public Command getDutyCycleCommand(Supplier<Double> dutyCycleSupplier) {
        return Commands.runEnd(() -> {
            turretAzimuth.setControl(new DutyCycleOut(dutyCycleSupplier.get()));
        }, () -> turretAzimuth.setControl(new NeutralOut()), this);
    }

    public Command getTurnToAngle(Rotation2d angle) {
        return Commands.runEnd(() -> {
            turretAzimuth.setControl(
                new DutyCycleOut(
                    azimuthController.calculate(getTurretRotation().getRotations(), angle.getRotations())
                )
            );
        }, () -> turretAzimuth.setControl(new NeutralOut()), this);
    }

    public Command getTurnToFieldRelativeAngle(Rotation2d angle) {
        return Commands.runEnd(() -> {
            Drivetrain drivetrain = Drivetrain.getInstance();
            double setpoint = (drivetrain.getRotation3d().getZ() / (Math.PI * 2)) - angle.getRotations();
            turretAzimuth.setControl(
                new DutyCycleOut(
                    azimuthController.calculate(getTurretRotation().getRotations(), setpoint)
                )
            );
        }, () -> turretAzimuth.setControl(new NeutralOut()), this);
    }

    private static Turret _instance;
    public static Turret getInstance() {
        if(_instance == null) _instance = new Turret();
        return _instance;
    }
}