// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Tunables;

public class Flywheels extends SubsystemBase {
  private final TalonFX leftTalon, rightTalon;
  private final PIDController velocityController = new PIDController(Tunables.FLYWHEEL_VELOCITY_PID_CONSTANTS.kP, Tunables.FLYWHEEL_VELOCITY_PID_CONSTANTS.kI, Tunables.FLYWHEEL_VELOCITY_PID_CONSTANTS.kP);

  /** Creates a new Flywheels. */
  public Flywheels() {
    leftTalon = new TalonFX(frc.robot.constants.Map.LEFT_FLYWHEEL);
    rightTalon = new TalonFX(frc.robot.constants.Map.RIGHT_FLYWHEEL);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flywheels/Velocity RPS", leftTalon.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Flywheels/Left Duty Cycle", leftTalon.get());
    SmartDashboard.putNumber("Flywheels/Right Duty Cycle", rightTalon.get());
    SmartDashboard.putData(this);
  }

  public Command dutyCycle(Supplier<Double> dutyCycleSupplier) {
    return Commands.runEnd(() -> {
      leftTalon.setControl(new DutyCycleOut(dutyCycleSupplier.get()));
      rightTalon.setControl(new DutyCycleOut(-dutyCycleSupplier.get()));
    }, () -> {
      leftTalon.setControl(new NeutralOut());
      rightTalon.setControl(new NeutralOut());
    }, this).withName("Duty Cycle");
  }

  public Command velocity(Supplier<Rotation2d> veloSupplierPerSecond) {
    return Commands.runEnd(() -> {
      double velocityDutyCycle = velocityController.calculate(leftTalon.getVelocity().getValueAsDouble(), veloSupplierPerSecond.get().getRotations());
      leftTalon.setControl(new DutyCycleOut(velocityDutyCycle));
      rightTalon.setControl(new DutyCycleOut(-velocityDutyCycle));
    }, () -> {
      leftTalon.setControl(new NeutralOut());
      rightTalon.setControl(new NeutralOut());
    }, this).withName("Velocity");
  }
}
