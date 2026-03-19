package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Configs;
import frc.robot.constants.Constraints;
import frc.robot.constants.Map;
import frc.robot.util.AlertContainer;

public class Feeder extends SubsystemBase {
    private final TalonFX towerTalon;
    private final TalonFX hotdogTalon;
    public Alert towerHighMotorTemp, towerCriticalMotorTemp, hotdogHighMotorTemp, hotdogCriticalMotorTemp;

    public Feeder() {
        towerTalon = new TalonFX(Map.TOWER_FEEDER);
        towerTalon.getConfigurator().apply(Configs.TOWER_FEEDER_CONFIGURATION);
        
        setupAlerts();
        hotdogTalon = new TalonFX(Map.HOTDOG_FEEDER);
        hotdogTalon.getConfigurator().apply(Configs.HOTDOG_FEEDER_CONFIGURATION);
    }

    public void setupAlerts() {
        towerHighMotorTemp = AlertContainer.getInstance().register(new Alert("FEEDER Tower Motor has a high tempurature", AlertType.kWarning));
        towerCriticalMotorTemp = AlertContainer.getInstance().register(new Alert("FEEDER Tower Motor has a critical tempurature", AlertType.kError));
        hotdogHighMotorTemp = AlertContainer.getInstance().register(new Alert("FEEDER Hotdog Motor has a high tempurature", AlertType.kWarning));
        hotdogCriticalMotorTemp = AlertContainer.getInstance().register(new Alert("FEEDER Hotdog Motor has a critical tempurature", AlertType.kError));
    }

    public void updateAlerts() {
        boolean towerHighTemp = towerTalon.getDeviceTemp().getValueAsDouble() > Constraints.KRAKEN_HIGH_TEMP;
        boolean towerCriticalTemp = towerTalon.getDeviceTemp().getValueAsDouble() > Constraints.KRAKEN_CRITICAL_TEMP;
        towerHighMotorTemp.set(towerHighTemp && !towerCriticalTemp);
        towerCriticalMotorTemp.set(towerCriticalTemp);

        boolean hotdogHighTemp = hotdogTalon.getDeviceTemp().getValueAsDouble() > Constraints.KRAKEN_HIGH_TEMP;
        boolean hotdogCriticalTemp = hotdogTalon.getDeviceTemp().getValueAsDouble() > Constraints.KRAKEN_CRITICAL_TEMP;
        hotdogHighMotorTemp.set(hotdogHighTemp && !hotdogCriticalTemp);
        hotdogCriticalMotorTemp.set(hotdogCriticalTemp);
    }

    @Override
    public void periodic() {
        updateAlerts();
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

    public boolean feeding() {
        return towerTalon.get() > 0.1 && hotdogTalon.get() > 0.1;
    }

    private static Feeder _instance;
    public static Feeder getInstance() {
        if(_instance == null) _instance = new Feeder();
        return _instance;
    }
}
