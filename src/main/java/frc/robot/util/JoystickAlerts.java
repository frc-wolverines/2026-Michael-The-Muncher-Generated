package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickAlerts extends SubsystemBase {
    public Trigger shiftEnding = new Trigger(() -> {
        double secondsRemaining = HubTracker.timeRemainingInCurrentShift().isPresent() ? HubTracker.timeRemainingInCurrentShift().get().in(Seconds) : 0.0;
        return secondsRemaining < 3;
    });
    public Trigger ourShift = new Trigger(HubTracker::isActive);
    public Trigger connected = new Trigger(() -> DriverStation.isDSAttached() && DriverStation.isJoystickConnected(0));

    public Command shiftStarting(CommandXboxController joystick) {
        return this.run(() -> {
            double secondsRemaining = HubTracker.timeRemainingInCurrentShift().isPresent() ? HubTracker.timeRemainingInCurrentShift().get().in(Seconds) : 0.0;
            double rumblePower = (3 - secondsRemaining) / 3;
            joystick.setRumble(RumbleType.kBothRumble, rumblePower);
        }).withTimeout(3)
            .andThen(this.run(() -> joystick.setRumble(RumbleType.kBothRumble, 0))).withTimeout(0.3)
            .andThen(this.run(() -> joystick.setRumble(RumbleType.kBothRumble, 1.0))).withTimeout(0.3)
            .andThen(this.run(() -> joystick.setRumble(RumbleType.kBothRumble, 0))).withTimeout(0.3)
            .andThen(this.run(() -> joystick.setRumble(RumbleType.kBothRumble, 1.0))).withTimeout(0.3)
            .finallyDo(() -> joystick.setRumble(RumbleType.kBothRumble, 0));
    }

    public Command shiftEnding(CommandXboxController joystick) {
        return this.run(() -> joystick.setRumble(RumbleType.kBothRumble, 1.0)).withTimeout(1)
            .andThen(this.run(() -> joystick.setRumble(RumbleType.kBothRumble, 0))).withTimeout(0.5)
            .andThen(this.run(() -> joystick.setRumble(RumbleType.kBothRumble, 1.0))).withTimeout(0.5)
            .andThen(this.run(() -> joystick.setRumble(RumbleType.kBothRumble, 0))).withTimeout(0.5)
            .andThen(this.run(() -> joystick.setRumble(RumbleType.kBothRumble, 1.0))).withTimeout(0.5)
            .finallyDo(() -> joystick.setRumble(RumbleType.kBothRumble, 0));
    }

    public Command connected(CommandXboxController joystick) {
        return this.run(() -> joystick.setRumble(RumbleType.kBothRumble, 1.0)).withTimeout(0.3)
            .andThen(this.run(() -> joystick.setRumble(RumbleType.kBothRumble, 0))).withTimeout(0.3)
            .andThen(this.run(() -> joystick.setRumble(RumbleType.kBothRumble, 1.0))).withTimeout(0.3)
            .finallyDo(() -> joystick.setRumble(RumbleType.kBothRumble, 0));
    }

    private static JoystickAlerts _instance;
    public static JoystickAlerts getInstance() {
        if(_instance == null) _instance = new JoystickAlerts();
        return _instance;
    }
}
