package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Configs;
import frc.robot.constants.Map;
import frc.robot.constants.Tunables;

public class Intake extends SubsystemBase {
    private final TalonFX intakePivot;
    private final PIDController intakePivotController = new PIDController(Tunables.INTAKE_PIVOT_PID_CONSTANTS.kP, Tunables.INTAKE_PIVOT_PID_CONSTANTS.kI, Tunables.INTAKE_PIVOT_PID_CONSTANTS.kD);
    
    public Intake() {
        intakePivot = new TalonFX(Map.INTAKE_PIVOT);
        intakePivot.getConfigurator().apply(Configs.INTAKE_PIVOT_CONFIGURATION);
        intakePivot.setPosition(0);
    }

    public Rotation2d getIntakeRotation() {
        return Rotation2d.fromRotations(intakePivot.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Pivot Angle Degrees", getIntakeRotation().getDegrees());
        SmartDashboard.putData(this);
    }

    public Command getDutyCycleCommand(Supplier<Double> pivotDutyCycleSupplier, Supplier<Double> intakeDutyCycleSupplier) {
        return Commands.runEnd(() -> {
            intakePivot.setControl(new DutyCycleOut(pivotDutyCycleSupplier.get()));
        }, () -> {
            intakePivot.setControl(new NeutralOut());
        }, this);
    }

    // public Command getIntakePivotCommand(double angle) {

    // }

    private static Intake _instance;
    public static Intake getInstance() {
        if(_instance == null) _instance = new Intake();
        return _instance;
    }
}