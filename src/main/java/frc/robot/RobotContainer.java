// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.core.io.IOContext;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Tunables;
import frc.robot.constants.Frames.IntakeState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController extraDebugJoystick = new CommandXboxController(1);
    private final Trigger intakeTrigger = new Trigger(() -> joystick.getLeftTriggerAxis() > 0.1);
    private final Trigger shootTrigger = new Trigger(() -> joystick.getRightTriggerAxis() > 0.9);
    private final Trigger spinUpTrigger = new Trigger(() -> joystick.getRightTriggerAxis() > 0.1).and(shootTrigger.negate());
    private final Trigger overrideFeeder = joystick.a();

    private final SendableChooser<Boolean> maintananceMode = new SendableChooser<>();

    public final Drivetrain drivetrain = Drivetrain.getInstance();
    public final Turret turret = Turret.getInstance();
    public final Intake intake = Intake.getInstance();
    public final Feeder feeder = Feeder.getInstance();
    public final Flywheels flywheels = Flywheels.getInstance();
    public final Lights lights = Lights.getInstance();
    public Trigger spunUp = new Trigger(() -> {
        double distance = drivetrain.getStateCopy().Pose.getTranslation().getDistance(turret.currentLandmark);
        double velocity = flywheels.getVelocityForDistance(distance);
        return Math.abs(velocity - flywheels.getVelocity()) < Tunables.FLYWHEEL_LAUNCH_TOLERANCE;
    });

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        drivetrain.resetRotation(Rotation2d.kZero);

        maintananceMode.addOption("False", false);
        maintananceMode.setDefaultOption("False", false);
        maintananceMode.addOption("True", true);
        maintananceMode.onChange(value -> {
            intake.setMaintananceMode(value);
        });

        SmartDashboard.putData("Maintanance Mode", maintananceMode);

        configureBindings();

        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {

        turret.setDefaultCommand(turret.turnToLandmark(FieldConstants.Hub.innerCenterPoint.toTranslation2d(), false));
        // extraDebugJoystick.a().onTrue(turret.getTurnToFieldRelativeAngle(Rotation2d.fromDegrees(0)));  
        // extraDebugJoystick.b().onTrue(turret.getTurnToFieldRelativeAngle(Rotation2d.fromDegrees(90)));  
        // extraDebugJoystick.y().onTrue(turret.getTurnToFieldRelativeAngle(Rotation2d.fromDegrees(180)));  
        // extraDebugJoystick.x().onTrue(turret.getTurnToFieldRelativeAngle(Rotation2d.fromDegrees(270))); 
        
        intakeTrigger.whileTrue(intake.down());
        feeder.setDefaultCommand(feeder.getDutyCycleCommand(() -> 0.0, () -> 0.0));
        shootTrigger.and(spunUp).or(overrideFeeder).whileTrue(feeder.getVoltageCommand(() -> 10.0, () -> 10.0));
        // shootTrigger.and(spunUp).and(intakeTrigger.negate()).whileTrue(intake.agitate().beforeStarting(Commands.waitSeconds(2)));

        joystick.b().onTrue(flywheels.runOnce(() -> {flywheels.cachedDebugVelocity = 0;}));
        joystick.x().onTrue(flywheels.dutyCycle(() -> 0.0));
        joystick.y().onTrue(flywheels.velocityFor(FieldConstants.Hub.innerCenterPoint.toTranslation2d()));
        joystick.pov(0).onTrue(flywheels.runOnce(() -> {flywheels.cachedDebugVelocity -= 5;}));
        joystick.pov(180).onTrue(flywheels.runOnce(() -> {flywheels.cachedDebugVelocity += 5;}));

        spinUpTrigger.or(shootTrigger).whileTrue(flywheels.velocityFor(FieldConstants.Hub.innerCenterPoint.toTranslation2d()));

        // intake.setDefaultCommand(
        //     intake.getStateCommand(() -> {
        //         return new IntakeState(Rotation2d.fromDegrees(extraDebugJoystick.getLeftTriggerAxis() > 0.1 ? 90 : 0.0), extraDebugJoystick.getLeftTriggerAxis() > 0.1 ? -10 : 0.0);
        //     })
        // );


        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        joystick.rightBumper().onTrue(drivetrain.runOnce(drivetrain::resetCommand));
        setupDrivetrain();

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void setupDrivetrain() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.65) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.65) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
