package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

public class Lights extends SubsystemBase {

    private final AddressableLED towerStrip;
    private final AddressableLEDBuffer towerBuffer;
    private final int LED_COUNT = 10;

    private final Trigger autoEnabled = new Trigger(DriverStation::isAutonomousEnabled);
    private final Trigger teleopEnabled = new Trigger(DriverStation::isTeleopEnabled);
    public final CommandXboxController joystick = new CommandXboxController(0);

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

        autoEnabled.whileTrue(autoIdle());
        teleopEnabled.whileTrue(teleopIdle());
        intaking.whileTrue(intaking());
        ableToShoot.whileTrue(ableToShoot());
        shooting.whileTrue(shooting());
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(this);
        SmartDashboard.putString("Lights/Color 1", towerBuffer.getLED(0).toString());
        SmartDashboard.putString("Lights/Color 2", towerBuffer.getLED(1).toString());
        SmartDashboard.putString("Lights/Color 3", towerBuffer.getLED(2).toString());
        SmartDashboard.putString("Lights/Color 4", towerBuffer.getLED(3).toString());
        SmartDashboard.putString("Lights/Color 5", towerBuffer.getLED(4).toString());
        SmartDashboard.putString("Lights/Color 6", towerBuffer.getLED(5).toString());
        SmartDashboard.putString("Lights/Color 7", towerBuffer.getLED(6).toString());
        SmartDashboard.putString("Lights/Color 8", towerBuffer.getLED(7).toString());
        SmartDashboard.putString("Lights/Color 9", towerBuffer.getLED(8).toString());
        SmartDashboard.putString("Lights/Color 10", towerBuffer.getLED(9).toString());
    }

    public Command idle() {
        return Commands.run(() -> {
            LEDPattern pattern = LEDPattern.solid(Color.kYellow).breathe(Seconds.of(1)).atBrightness(Percent.of(70));
            pattern.applyTo(towerBuffer);
            towerStrip.setData(towerBuffer);
        }, this).ignoringDisable(true).withName("Disabled & Idle");
    }

    public Command autoIdle() {
        return Commands.run(() -> {
            LEDPattern pattern = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlue, Color.kWhite).scrollAtRelativeSpeed(Percent.per(Second).of(50)).atBrightness(Percent.of(70));
            pattern.applyTo(towerBuffer);
            towerStrip.setData(towerBuffer);
        }, this).withName("Enabled & Idle");
    }

    public Command teleopIdle() {
        return Commands.run(() -> {
            LEDPattern pattern = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kYellow, Color.kWhite).scrollAtRelativeSpeed(Percent.per(Second).of(50)).atBrightness(Percent.of(70));
            pattern.applyTo(towerBuffer);
            towerStrip.setData(towerBuffer);
        }, this).withName("Enabled & Idle");
    }

    public Command intaking() {
        return Commands.run(() -> {
            LEDPattern pattern = LEDPattern.solid(Color.kBlue).blink(Seconds.of(0.1)).atBrightness(Percent.of(70));
            pattern.applyTo(towerBuffer);
            towerStrip.setData(towerBuffer);
        }, this).withName("Intaking");
    }

    public Command ableToShoot() {
        return Commands.run(() -> {
            LEDPattern pattern = LEDPattern.solid(Color.kLimeGreen).atBrightness(Percent.of(70));
            pattern.applyTo(towerBuffer);
            towerStrip.setData(towerBuffer);
        }, this).withName("Turret Aligned");
    }

    public Command shooting() {
        return Commands.run(() -> {
            LEDPattern pattern = LEDPattern.solid(Color.kLimeGreen).blink(Seconds.of(0.1)).atBrightness(Percent.of(70));
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