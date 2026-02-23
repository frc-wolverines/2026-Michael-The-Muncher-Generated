package frc.robot.constants;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Frames.IntakeState;

public class Tunables {
    public static final PIDConstants TURRET_AZIMUTH_PID_CONSTANTS = new PIDConstants(6.0);
    public static final PIDConstants INTAKE_PIVOT_PID_CONSTANTS = new PIDConstants(2.5);

    public static final Rotation2d INTAKE_DOWN_ROTATION = Rotation2d.fromDegrees(0);
    public static final Rotation2d INTAKE_UP_ROTATION = Rotation2d.fromDegrees(0);
    public static final Rotation2d INTAKE_AGITATE_ROTATION = Rotation2d.fromDegrees(0);

    public static final Voltage INTAKE_ADVANCE_VOLTAGE = Voltage.ofBaseUnits(-10.0, Volts);
    public static final Voltage INTAKE_RETREAT_VOLTAGE = Voltage.ofBaseUnits(10.0, Volts);
    public static final Voltage INTAKE_IDLE_VOLTAGE = Voltage.ofBaseUnits(0.0, Volts);

    public static final IntakeState INTAKE_DOWN_STATE = new IntakeState(INTAKE_DOWN_ROTATION, INTAKE_ADVANCE_VOLTAGE);
    public static final IntakeState INTAKE_UP_STATE = new IntakeState(INTAKE_UP_ROTATION, INTAKE_IDLE_VOLTAGE);
    public static final IntakeState INTAKE_AGITATE_STATE = new IntakeState(INTAKE_AGITATE_ROTATION, INTAKE_IDLE_VOLTAGE);
}