// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.ejml.dense.row.decomposition.hessenberg.TridiagonalDecompositionHouseholderOrig_DDRM;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.core.io.IOContext;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.jni.WPIMathJNI;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.subsystems.OldIntka;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Intake.IntakePivotState;
import frc.robot.util.CustomMath;
import frc.robot.util.JoystickAlerts;

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

    public final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController extraDebugJoystick = new CommandXboxController(1);

    //CONTROL TRIGGERS
    private final Trigger intakeTrigger = new Trigger(() -> joystick.getLeftTriggerAxis() > 0.1);
    private final Trigger outtakeTrigger = new Trigger(joystick.leftBumper());
    private final Trigger bringUpIntakeTrigger = new Trigger(joystick.rightStick());

    private final Trigger shootTrigger = new Trigger(() -> joystick.getRightTriggerAxis() > 0.9);
    private final Trigger spinUpTrigger = new Trigger(() -> joystick.getRightTriggerAxis() > 0.1).and(shootTrigger.negate());

    private final Trigger overrideFeeder = joystick.x();

    private final SendableChooser<Boolean> maintananceMode = new SendableChooser<>();

    public final Drivetrain drivetrain = Drivetrain.getInstance();
    public final Turret turret = Turret.getInstance();
    public final Intake intake = Intake.getInstance();
    public final Feeder feeder = Feeder.getInstance();
    public final Flywheels flywheels = Flywheels.getInstance();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Intake", 
            intake.holdStateWithActiveRollers(-0.65)
                .beforeStarting(intake.setState(IntakePivotState.DOWN)));
        NamedCommands.registerCommand("Shoot", shootCommand());
        autoChooser = AutoBuilder.buildAutoChooser("None");
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
        //INTAKE
        intakeTrigger.whileTrue(
            intake.holdStateWithActiveRollers(-0.65)
                .beforeStarting(intake.setState(IntakePivotState.DOWN)));
        outtakeTrigger.whileTrue(
            intake.holdStateWithActiveRollers(1).alongWith(feeder.getDutyCycleCommand(() -> -1.0, () -> 1.0))
                .beforeStarting(intake.setState(IntakePivotState.DOWN)));
        bringUpIntakeTrigger.onTrue(intake.setState(IntakePivotState.UP));

        //TURRET
        joystick.leftStick().toggleOnTrue(turret.lock());
        joystick.y().onTrue(turret.zeroTurret());

        //FEEDER
        overrideFeeder.whileTrue(feeder.getDutyCycleCommand(() -> 1.0, () -> -1.0 * (joystick.a().getAsBoolean() ? -1.0 : 1.0)));

        //FLYWHEELS
        spinUpTrigger.whileTrue(flywheels.velocityFor(FieldConstants.Hub.innerCenterPoint.toTranslation2d()));

        //COMPOSITE
        shootTrigger.whileTrue(shootCommand());

        //DRIVETRAIN
        joystick.pov(0).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        joystick.rightBumper().onTrue(drivetrain.runOnce(drivetrain::resetCommand));
        setupDrivetrain();
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void setupDrivetrain() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * (shootTrigger.getAsBoolean() ? 0.2 : 1)) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * (shootTrigger.getAsBoolean() ? 0.2 : 1)) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
    }

    public Command shootCommand() {
        return Commands.parallel(
            flywheels.velocityFor(CustomMath.makeTranslationAllianceRelative(FieldConstants.Hub.innerCenterPoint.toTranslation2d())),
            feeder.getDutyCycleCommand(() -> 1.0, () -> -1.0),
            intake.holdStateWithActiveRollersSlow(0.0)
                    .beforeStarting(intake.setState(IntakePivotState.UP))
        );
    }

    public Command getAutonomousCommand() {
        return new WaitCommand(SmartDashboard.getNumber("Autonomous Starting Delay", 0.0)).andThen(autoChooser.getSelected().asProxy());
    }
}