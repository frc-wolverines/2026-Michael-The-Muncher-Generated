package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;

import java.lang.Thread.State;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.event.NetworkBooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Configs;
import frc.robot.constants.Map;
import frc.robot.constants.Tunables;
import frc.robot.constants.Frames.IntakeState;

public class Intake extends SubsystemBase {
    private final TalonFX intakePivot;
    private final PIDController intakePivotController = new PIDController(Tunables.INTAKE_PIVOT_PID_CONSTANTS.kP, Tunables.INTAKE_PIVOT_PID_CONSTANTS.kI, Tunables.INTAKE_PIVOT_PID_CONSTANTS.kD);

    private final TalonFX intakeRollers;
    
    public Intake() {
        intakePivot = new TalonFX(Map.INTAKE_PIVOT);
        intakePivot.getConfigurator().apply(Configs.INTAKE_PIVOT_CONFIGURATION);
        intakePivot.setPosition(0);

        intakeRollers = new TalonFX(Map.INTAKE_ROLLERS);
        intakeRollers.getConfigurator().apply(Configs.INTAKE_ROLLERS_CONFIGURATION);
    }

    public Rotation2d getIntakeRotation() {
        return Rotation2d.fromRotations(intakePivot.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Pivot Angle Degrees", getIntakeRotation().getDegrees());
        SmartDashboard.putNumber("Intake/Roller Current Stator", intakeRollers.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller Voltage", intakeRollers.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putData(this);
    }

    public Command getDutyCycleCommand(Supplier<Double> pivotDutyCycleSupplier, Supplier<Double> intakeDutyCycleSupplier) {
        return Commands.runEnd(() -> {
            intakePivot.setControl(new DutyCycleOut(pivotDutyCycleSupplier.get()));
            intakeRollers.setControl(new VoltageOut(intakeDutyCycleSupplier.get() * 10));
        }, () -> {
            intakePivot.setControl(new NeutralOut());
            intakeRollers.setControl(new NeutralOut());
        }, this);
    }

    public Command getStateCommand(Supplier<IntakeState> stateSupplier) {
        return Commands.runEnd(() -> {
            IntakeState state = stateSupplier.get();
            intakePivot.setControl(new DutyCycleOut(intakePivotController.calculate(getIntakeRotation().getRotations(), state.rotation().getRotations())));
            intakeRollers.setControl(new VoltageOut(state.voltage()));
        }, () -> {
            intakePivot.setControl(new NeutralOut());
            intakeRollers.setControl(new NeutralOut());
        }, this);
    }

    private static Intake _instance;
    public static Intake getInstance() {
        if(_instance == null) _instance = new Intake();
        return _instance;
    }
}