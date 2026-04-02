package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import org.w3c.dom.css.RGBColor;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.hal.PWMJNI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.constants.Tunables;
import frc.robot.util.AlertContainer;

public class Lights extends SubsystemBase {

    // private final AddressableLED towerStrip;
    // private final AddressableLEDBuffer towerBuffer;
    // private final int LED_COUNT = 55;

    // private final Trigger idleConnected = new Trigger(DriverStation::isDSAttached);
    // private final Trigger autoEnabled = new Trigger(DriverStation::isAutonomousEnabled);
    // private final Trigger teleopEnabled = new Trigger(DriverStation::isTeleopEnabled);
    // private final Trigger majorFaultPresent = new Trigger(() -> !AlertContainer.getInstance().getMajorFaults().isEmpty());

    // //Criteria met when the intake is at the correct angle and the rollers are running at an intaking speed to intake a ball
    // private final Trigger intaking;

    // //Criteria met when the turret is aligned with the target and ready to shoot
    // private final Trigger ableToShoot;

    // //Criteria met when the shooter wheel is up to speed, the turret is aligned with the target, and the feeding system is running to shoot a ball
    // private final Trigger shooting;

    // public Lights() {
    //     towerStrip = new AddressableLED(1);
    //     towerBuffer = new AddressableLEDBuffer(LED_COUNT);
    //     towerStrip.setLength(towerBuffer.getLength());

    //     towerStrip.setData(towerBuffer);
    //     towerStrip.start();
    //     setDefaultCommand(idle());

    //     intaking = new Trigger(() -> Intake.getInstance().state.equals(Tunables.INTAKE_DOWN_WITH_ADVANCE_STATE));
    //     ableToShoot = new Trigger(() -> Turret.getInstance().pointedTowardsTarget());
    //     shooting = new Trigger(() -> Feeder.getInstance().feeding() && Flywheels.getInstance().spunUp());
        
    //     idleConnected.negate().whileTrue(idleNotConnected());
    //     autoEnabled.and(intaking.negate()).and(ableToShoot.negate()).and(shooting.negate()).whileTrue(autoIdle());
    //     teleopEnabled.and(intaking.negate()).and(ableToShoot.negate()).and(shooting.negate()).whileTrue(teleopIdle());
    //     ableToShoot.and(ableToShoot.negate()).and(shooting.negate()).whileTrue(ableToShoot());
    //     intaking.and(shooting.negate()).whileTrue(intaking());
    //     shooting.whileTrue(shooting());
    //     majorFaultPresent.whileTrue(error());
    // }

    // @Override
    // public void periodic() {
    //     SmartDashboard.putData(this);
    //     towerStrip.setData(towerBuffer);

    //     SmartDashboard.putBoolean("LEDs Connected", PWMJNI.checkPWMChannel(0));

    //     SmartDashboard.putString("Lights/Color 1", towerBuffer.getLED(0).toString());
    // }

    // public Command idleNotConnected() {
    //     return Commands.run(() -> {
    //         LEDPattern pattern = LEDPattern.solid(Color.kPurple).breathe(Seconds.of(1)).atBrightness(Percent.of(70));
    //         pattern.applyTo(towerBuffer);
    //     }, this).ignoringDisable(true).withName("Disabled & Idle");
    // }

    // public Command error() {
    //     return Commands.run(() -> {
    //         LEDPattern pattern = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.1)).atBrightness(Percent.of(70));
    //         pattern.applyTo(towerBuffer);
    //     }, this).ignoringDisable(true).withName("Error");
    // }

    // public Command idle() {
    //     return Commands.run(() -> {
    //         LEDPattern pattern = LEDPattern.solid(new Color(255, 150,0)).breathe(Seconds.of(1)).atBrightness(Percent.of(70));
    //         pattern.applyTo(towerBuffer);
    //     }, this).ignoringDisable(true).withName("Disabled & Idle");
    // }

    // public Command autoIdle() {
    //     return Commands.run(() -> {
    //         LEDPattern pattern = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlue, Color.kWhite).scrollAtRelativeSpeed(Percent.per(Second).of(50)).atBrightness(Percent.of(70));
    //         pattern.applyTo(towerBuffer);
    //     }, this).withName("Enabled & Idle");
    // }

    // public Command teleopIdle() {
    //     return Commands.run(() -> {
    //         LEDPattern pattern = LEDPattern.gradient(GradientType.kDiscontinuous, new Color(255, 150,0), Color.kBlack).scrollAtRelativeSpeed(Percent.per(Second).of(50)).atBrightness(Percent.of(70));
    //         pattern.applyTo(towerBuffer);
    //     }, this).withName("Enabled & Idle");
    // }

    // public Command intaking() {
    //     return Commands.run(() -> {
    //         LEDPattern pattern = LEDPattern.solid(Color.kBlue).blink(Seconds.of(0.1)).atBrightness(Percent.of(70));
    //                     pattern.applyTo(towerBuffer);

    //     }, this).withName("Intaking");
    // }

    // public Command ableToShoot() {
    //     return Commands.run(() -> {
    //         LEDPattern pattern = LEDPattern.solid(Color.kGreen).atBrightness(Percent.of(70));
    //         pattern.applyTo(towerBuffer);

    //     }, this).withName("Turret Aligned");
    // }

    // public Command shooting() {
    //     return Commands.run(() -> {
    //         LEDPattern pattern = LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.1)).atBrightness(Percent.of(70));
    //         pattern.applyTo(towerBuffer);
    //     }, this).withName("Shooting");
    // }
    
    // private static Lights _instance;
    // public static Lights getInstance() {
    //     if(_instance == null) _instance = new Lights();
    //     return _instance;
    // }
}