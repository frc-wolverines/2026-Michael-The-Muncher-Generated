package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CustomMath;

public class Intake extends SubsystemBase {
    private TalonFX leftRoller, rightRoller, pivot;
    private PIDController controller;
    public IntakePivotState _state = IntakePivotState.UP;

    public Intake() {
        leftRoller = new TalonFX(31);
        rightRoller = new TalonFX(30);
        pivot = new TalonFX(32);
        pivot.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(30).withStatorCurrentLimitEnable(true));
        pivot.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(3 * 9 * (24/15)));
        pivot.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        pivot.setPosition(0);
        leftRoller.setControl(new Follower(30, MotorAlignmentValue.Opposed));

        controller = new PIDController(1.5, 0, 0);
        setDefaultCommand(holdState());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Pivot Position", getRotation().getRotations());
        SmartDashboard.putString("Intake/State", _state.toString());
        SmartDashboard.putData(this);
        SmartDashboard.putNumber("Intake/Left Stator Current", leftRoller.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Right Stator Current", rightRoller.getStatorCurrent().getValueAsDouble());
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromRotations(pivot.getPosition().getValueAsDouble());
    }

    public void setMaintananceMode(boolean enabled) {
        pivot.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake));
    }

    public Command holdState() {
        return Commands.runEnd(this::pivotToStateRotation, this::stop, this).withName("Hold State: " + _state);
    }

    public Command setState(IntakePivotState state) {
        return new InstantCommand(() -> _state = state, this).withName("Set State: " + state);
    }

    public Command holdStateWithActiveRollers(double rollDutyCycle) {
        return Commands.runEnd(() -> {
            pivotToStateRotation();
            rightRoller.setControl(new DutyCycleOut(rollDutyCycle));
        }, this::stop, this).withName("Hold State With Active Rollers");
    }

    public Command holdStateWithActiveRollersSlow(double rollDutyCycle) {
        return Commands.runEnd(() -> {
            pivotToStateRotation(0.1);
            rightRoller.setControl(new DutyCycleOut(rollDutyCycle));
        }, this::stop, this).withName("Hold State With Active Rollers");
    } 

    public void pivotToStateRotation() {
        pivot.setControl(new DutyCycleOut(controller.calculate(getRotation().getRotations(), _state.rotation.getRotations())));
    }

    public void pivotToStateRotation(double dutyCycleCap) {
        pivot.setControl(new DutyCycleOut(
            MathUtil.clamp(controller.calculate(getRotation().getRotations(), _state.rotation.getRotations()), -dutyCycleCap, dutyCycleCap)
        ));
    }

    public Command dutyCycle(Supplier<Double> dtSupplier, Supplier<Double> pivotPower) {
        return Commands.runEnd(() -> {
            rightRoller.setControl(new DutyCycleOut(dtSupplier.get() * 0.65));
            pivot.setControl(new DutyCycleOut(pivotPower.get()));
        }, () -> {
            rightRoller.stopMotor();
            pivot.stopMotor();
        }, this);
    }
    
    public void stop() {
        pivot.stopMotor();
        rightRoller.stopMotor();
    }

    public Command position(Rotation2d rotation, double speed) {
        return Commands.runEnd(() -> {
            pivot.setControl(new DutyCycleOut(controller.calculate(getRotation().getRotations(), rotation.getRotations())));
            rightRoller.setControl(new DutyCycleOut(speed));
        }, () -> {
            pivot.stopMotor();
            rightRoller.stopMotor();
        }, this);
    }

    public enum IntakePivotState {
        DOWN(Rotation2d.kZero),
        UP(Rotation2d.fromRotations(0.4));

        public final Rotation2d rotation;
        private IntakePivotState(Rotation2d rotation2d) {
            this.rotation = rotation2d;
        }
    }

    private static Intake _instance;
    public static Intake getInstance() {
        if(_instance == null) _instance = new Intake(); 
        return _instance;
    }
}
