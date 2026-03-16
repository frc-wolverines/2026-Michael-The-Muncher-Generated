package frc.robot.constants;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Configs {
    public static final TalonFXConfiguration TURRET_AZIMUTH_CONFIGURATION = new TalonFXConfiguration()
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(Constraints.TURRET_AZIMUTH_GEAR_RATIO))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
        )
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(30)
        );

    public static final TalonFXConfiguration INTAKE_PIVOT_CONFIGURATION = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(20)
                .withStatorCurrentLimitEnable(true));

    public static final TalonFXConfiguration INTAKE_ROLLERS_CONFIGURATION = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast));

    public static final TalonFXConfiguration HOTDOG_FEEDER_CONFIGURATION = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
        );

    public static final TalonFXConfiguration TOWER_FEEDER_CONFIGURATION = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
        );
}