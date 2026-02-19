package frc.robot.constants;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Configs {
    public static final TalonFXConfiguration TURRET_AZIMUTH_CONFIGURATION = new TalonFXConfiguration()
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(TrueConstants.TURRET_AZIMUTH_GEAR_RATIO))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
        );

    public static final TalonFXConfiguration INTAKE_PIVOT_CONFIGURATION = new TalonFXConfiguration()
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(TrueConstants.INTAKE_PIVOT_GEAR_RATIO))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake));

    public static final TalonFXConfiguration INTAKE_MOTOR_CONFIGURATION = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast));
}