package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Configs;
import frc.robot.constants.Map;

public class Feeder extends SubsystemBase {
    private final TalonFX towerTalon;
    private final TalonFX hotdogTalon;

    public Feeder() {
        towerTalon = new TalonFX(Map.TOWER_FEEDER);
        towerTalon.getConfigurator().apply(Configs.TOWER_FEEDER_CONFIGURATION);
        
        hotdogTalon = new TalonFX(Map.HOTDOG_FEEDER);
        hotdogTalon.getConfigurator().apply(Configs.HOTDOG_FEEDER_CONFIGURATION);
    }

    public Command getDutyCycleCommand(Supplier<Double> towerDutyCycle, Supplier<Double> hotdogDutyCycle) {
        return Commands.runEnd(() -> {
            towerTalon.setControl(new DutyCycleOut(towerDutyCycle.get()));
            hotdogTalon.setControl(new DutyCycleOut(hotdogDutyCycle.get()));
        }, this::stop, this);
    }

    public Command getVoltageCommand(Supplier<Double> towerVoltage, Supplier<Double> hotdogVoltage) {
        return Commands.runEnd(() -> {
            towerTalon.setControl(new VoltageOut(towerVoltage.get()));
            hotdogTalon.setControl(new VoltageOut(hotdogVoltage.get()));
        }, this::stop, this);
    }

    public void stop() {
        towerTalon.setControl(new NeutralOut());
        hotdogTalon.setControl(new NeutralOut());
    }

    private static Feeder _instance;
    public static Feeder getInstance() {
        if(_instance == null) _instance = new Feeder();
        return _instance;
    }
}
