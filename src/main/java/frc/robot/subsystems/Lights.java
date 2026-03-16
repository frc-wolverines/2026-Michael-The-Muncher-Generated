package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Lights extends SubsystemBase {

    private final AddressableLED towerStrip;
    private final AddressableLEDBuffer towerBuffer;
    private final int LED_COUNT = 10;

    private final Trigger enabled = new Trigger(DriverStation::isEnabled);
    private final Trigger intaking = new Trigger(() -> false);
    private final Trigger ableToShoot = new Trigger(() -> false);
    private final Trigger shooting = new Trigger(() -> false);

    public Lights() {
        towerStrip = new AddressableLED(0);
        towerBuffer = new AddressableLEDBuffer(LED_COUNT);
        towerStrip.setLength(LED_COUNT);

        towerStrip.setData(towerBuffer);
        towerStrip.start();
        setDefaultCommand(idle());

        enabled.whileTrue(enabledIdle());
        intaking.whileTrue(intaking());
        ableToShoot.whileTrue(ableToShoot());
        shooting.whileTrue(shooting());
    }

    public Command idle() {
        return Commands.run(() -> {
            LEDPattern pattern = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kYellow, Color.kLightGoldenrodYellow).scrollAtRelativeSpeed(Percent.per(Second).of(25));
            pattern.applyTo(towerBuffer);
            towerStrip.setData(towerBuffer);
        }, this).ignoringDisable(true).withName("Disabled & Idle");
    }

    public Command enabledIdle() {
        return Commands.run(() -> {
            LEDPattern pattern = LEDPattern.solid(Color.kYellow).breathe(Seconds.of(1));
            pattern.applyTo(towerBuffer);
            towerStrip.setData(towerBuffer);
        }, this).withName("Enabled & Idle");
    }

    public Command intaking() {
        return Commands.run(() -> {
            LEDPattern pattern = LEDPattern.solid(Color.kBlue).blink(Seconds.of(0.1));
            pattern.applyTo(towerBuffer);
            towerStrip.setData(towerBuffer);
        }, this).withName("Intaking");
    }

    public Command ableToShoot() {
        return Commands.run(() -> {
            LEDPattern pattern = LEDPattern.solid(Color.kLimeGreen).blink(Seconds.of(0.1));
            pattern.applyTo(towerBuffer);
            towerStrip.setData(towerBuffer);
        }, this).withName("Turret Aligned");
    }

    public Command shooting() {
        return Commands.run(() -> {
            LEDPattern pattern = LEDPattern.solid(Color.kGreen);
            pattern.applyTo(towerBuffer);
            towerStrip.setData(towerBuffer);
        }, this).withName("Shooting");
    }
    
    private static Lights _instance;
    public static Lights getInstance() {
        if(_instance == null) _instance = new Lights();
        return _instance;
    }
}