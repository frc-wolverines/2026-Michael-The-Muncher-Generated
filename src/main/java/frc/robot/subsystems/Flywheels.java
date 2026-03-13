// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Tunables;

public class Flywheels extends SubsystemBase {
  private final TalonFX leftTalon, rightTalon;
  public double cachedDebugVelocity = 0.0;

  /** Creates a new Flywheels. */
  public Flywheels() {
    leftTalon = new TalonFX(frc.robot.constants.Map.LEFT_FLYWHEEL);
    leftTalon.getConfigurator().apply(Tunables.FLYWHEEL_SLOT_ZEROS);
    rightTalon = new TalonFX(frc.robot.constants.Map.RIGHT_FLYWHEEL);
    rightTalon.getConfigurator().apply(Tunables.FLYWHEEL_SLOT_ZEROS);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flywheels/Velocity RPS", leftTalon.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Flywheels/Left Duty Cycle", leftTalon.get());
    SmartDashboard.putNumber("Flywheels/Right Duty Cycle", rightTalon.get());
    SmartDashboard.putData(this);
  }

  public double getVelocity() {
    return leftTalon.getVelocity().getValueAsDouble(); 
  }

  public Command dutyCycle(Supplier<Double> dutyCycleSupplier) {
    return Commands.runEnd(() -> {
      leftTalon.setControl(new DutyCycleOut(dutyCycleSupplier.get()));
      rightTalon.setControl(new DutyCycleOut(dutyCycleSupplier.get()));
    }, () -> {
      leftTalon.setControl(new NeutralOut());
      rightTalon.setControl(new NeutralOut());
    }, this).withName("Duty Cycle");
  }

  public Command velocity(Supplier<Rotation2d> veloSupplierPerSecond) {
    return Commands.runEnd(() -> {
      leftTalon.setControl(new VelocityDutyCycle(veloSupplierPerSecond.get().getRotations()));
      rightTalon.setControl(new VelocityDutyCycle(-veloSupplierPerSecond.get().getRotations()));
    }, () -> {
      leftTalon.setControl(new NeutralOut());
      rightTalon.setControl(new NeutralOut());
    }, this).withName("Velocity");
  }

  public Command velocityFor(Translation2d landmark) {
            return Commands.runEnd(() -> {
              Translation2d newLandmark = landmark;
              Drivetrain drivetrain = Drivetrain.getInstance();
              SwerveDriveState state = drivetrain.getStateCopy();
              Pose2d robotPose = state.Pose;
              double distance = robotPose.getTranslation().getDistance(newLandmark);
              double velocity = getVelocityForDistance(distance);
              leftTalon.setControl(new VelocityDutyCycle(velocity));
              rightTalon.setControl(new VelocityDutyCycle(-velocity));
            }, () -> {
              leftTalon.setControl(new NeutralOut());
              rightTalon.setControl(new NeutralOut());
            }, this).withName("Landmark Velocity");
  }

  public double getVelocityForDistance(double distanceMeters) {
    return (-0.357347 * distanceMeters * distanceMeters) - (6.0486 * distanceMeters) - 31.85358;
  }

  private static Flywheels _instance;
  public static Flywheels getInstance() {
    if(_instance == null) _instance = new Flywheels();
    return _instance;
  }
}
