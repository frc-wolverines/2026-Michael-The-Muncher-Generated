// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  private PIDController turretAzimuthController = new PIDController(0.1, 0, 0);
  private final double rootMeasurementConversion = 1 / (5 * 5 * 10);
  private TalonFX turretAzimuth;

  public Turret() {
    turretAzimuth = new TalonFX(14);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private static Turret _instance;
  public static Turret getInstance() {
    if(_instance == null) _instance = new Turret();
    return _instance;
  }
}
