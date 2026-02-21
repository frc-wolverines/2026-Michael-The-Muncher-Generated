package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.config.PIDConstants;

public class Tunables {
    public static final PIDConstants TURRET_AZIMUTH_PID_CONSTANTS = new PIDConstants(6.0);
    public static final PIDConstants INTAKE_PIVOT_PID_CONSTANTS = new PIDConstants(2.5);
}