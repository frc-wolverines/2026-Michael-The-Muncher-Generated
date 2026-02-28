package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;

import java.lang.Thread.State;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.event.NetworkBooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Configs;
import frc.robot.constants.Constraints;
import frc.robot.constants.Map;
import frc.robot.constants.Tunables;
import frc.robot.constants.Frames.IntakeState;

public class Intake extends SubsystemBase {
    private final TalonFX intakePivot;
    private final PIDController intakePivotController = new PIDController(Tunables.INTAKE_PIVOT_PID_CONSTANTS.kP, Tunables.INTAKE_PIVOT_PID_CONSTANTS.kI, Tunables.INTAKE_PIVOT_PID_CONSTANTS.kD);
    private final DutyCycleEncoder intakeEncoder;
    private final SlewRateLimiter rollerAccelLimiter = new SlewRateLimiter(10);

    private final TalonFX intakeRollers;
    
    public Intake() {
        intakePivot = new TalonFX(Map.INTAKE_PIVOT);
        intakePivot.getConfigurator().apply(Configs.INTAKE_PIVOT_CONFIGURATION);
        intakePivot.setPosition(0);

        intakeEncoder = new DutyCycleEncoder(0);

        intakeRollers = new TalonFX(Map.INTAKE_ROLLERS);
        intakeRollers.getConfigurator().apply(Configs.INTAKE_ROLLERS_CONFIGURATION);

        // setDefaultCommand(idle());
    }

    public Rotation2d getIntakeRotation() {
        return Rotation2d.fromDegrees((intakeEncoder.get() * 360 - Constraints.INTAKE_ENCODER_OFFSET));   
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Pivot Angle Degrees", getIntakeRotation().getDegrees());
        SmartDashboard.putNumber("Intake/Roller Current Stator", intakeRollers.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller Voltage", intakeRollers.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putData(this);
    }

    private Command getDutyCycleCommand(Supplier<Double> pivotDutyCycleSupplier, Supplier<Double> intakeDutyCycleSupplier) {
        return Commands.runEnd(() -> {
            intakePivot.setControl(new DutyCycleOut(pivotDutyCycleSupplier.get()));
            intakeRollers.setControl(new VoltageOut(intakeDutyCycleSupplier.get() * 10));
        }, () -> {
            intakePivot.setControl(new NeutralOut());
            intakeRollers.setControl(new NeutralOut());
        }, this);
    }

    private Command getStateCommand(IntakeState state) {
        return Commands.runEnd(() -> {
            intakePivot.setControl(new DutyCycleOut(intakePivotController.calculate(getIntakeRotation().getRotations(), state.rotation().getRotations())));
            intakeRollers.setControl(new VoltageOut(rollerAccelLimiter.calculate(state.voltage().baseUnitMagnitude())));
        }, () -> {
            intakePivot.setControl(new NeutralOut());
            intakeRollers.setControl(new NeutralOut());
        }, this);
    }

    public Command idle() {
        return getStateCommand(Tunables.INTAKE_UP_STATE);
    }

    public Command agitate() {
        return Commands.run(() -> {
            intakePivot.setControl(new DutyCycleOut(intakePivotController.calculate(getIntakeRotation().getRotations(), Tunables.INTAKE_UP_ROTATION.getRotations())));
        }, this)
            .withTimeout(0.5)
            .andThen(Commands.run(() -> {
                intakePivot.setControl(new DutyCycleOut(intakePivotController.calculate(getIntakeRotation().getRotations(), Tunables.INTAKE_AGITATE_ROTATION.getRotations())));
            }, this))
            .withTimeout(0.5).repeatedly().finallyDo(this::stop);
    }

    public Command down() {
        return getStateCommand(Tunables.INTAKE_DOWN_STATE);
    }

    public void stop() {
        intakePivot.setControl(new NeutralOut());
        intakeRollers.setControl(new NeutralOut());
    }

    public void setMaintananceMode(boolean enabled) {
        intakePivot.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake));
    }

    private static Intake _instance;
    public static Intake getInstance() {
        if(_instance == null) _instance = new Intake();
        return _instance;
    }
}