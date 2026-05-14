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
import edu.wpi.first.math.controller.PIDController;
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
    public final CommandXboxController extraDebugJoystick = new CommandXboxController(1);
    private final PIDController rotationController = new PIDController(7.0, 0, 0);

    //CONTROL TRIGGERS
    private final Trigger intakeTrigger = new Trigger(() -> joystick.getLeftTriggerAxis() > 0.1);
    private final Trigger outtakeTrigger = new Trigger(joystick.leftBumper());
    private final Trigger bringUpIntakeTrigger = new Trigger(joystick.leftStick());

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
        NamedCommands.registerCommand("Shoot", autoShootCommand());
        autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto Mode", autoChooser);

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
        intakeTrigger.and(extraDebugJoystick.a().negate()).whileTrue(
            intake.holdStateWithActiveRollers(-0.65)
                .beforeStarting(intake.setState(IntakePivotState.DOWN)));
        outtakeTrigger.whileTrue(
            intake.holdStateWithActiveRollers(1).alongWith(feeder.getDutyCycleCommand(() -> -1.0, () -> 1.0))
                .beforeStarting(intake.setState(IntakePivotState.DOWN)));
        bringUpIntakeTrigger.and(extraDebugJoystick.a().negate()).onTrue(intake.setState(IntakePivotState.UP));

        //TURRET
        // joystick.y().onTrue(turret.zeroTurret());

        //FEEDER
        overrideFeeder.and(extraDebugJoystick.a().negate()).whileTrue(feeder.getDutyCycleCommand(() -> 1.0, () -> -1.0 * (joystick.a().getAsBoolean() ? -1.0 : 1.0)));

        //FLYWHEELS
        // spinUpTrigger.whileTrue(flywheels.velocityFor(FieldConstants.Hub.innerCenterPoint.toTranslation2d()));

        //COMPOSITE
        shootTrigger.whileTrue(shootFixedVelocityCommand());
        extraDebugJoystick.rightBumper().whileTrue(shootFixedVelocityBigCommand());

        //DRIVETRAIN
        joystick.pov(0).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        joystick.rightStick().onTrue(drivetrain.runOnce(drivetrain::resetCommand));
        setupDrivetrain();
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void setupDrivetrain() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                SwerveDriveState state = drivetrain.getState();
                Pose2d robotPose = CustomMath.makePoseAllianceRelative(state.Pose);
                Rotation2d translationAngle = CustomMath.makeTranslationAllianceRelative(FieldConstants.Hub.innerCenterPoint.toTranslation2d()).minus(CustomMath.makeTranslationAllianceRelative(robotPose.getTranslation())).getAngle();
                double rotation = ((-joystick.getRightX() * MaxAngularRate) * 0.65 * (extraDebugJoystick.a().getAsBoolean() ? 0.0 : 1.0)) + (-extraDebugJoystick.getRightX() * MaxAngularRate);
                if(Math.abs(rotation) < 0.1 && (joystick.a().getAsBoolean())) {
                    rotation = -rotationController.calculate(robotPose.getRotation().getRotations(), translationAngle.getRotations());
                }

                double x = -joystick.getLeftY() * MaxSpeed * 0.25 * (extraDebugJoystick.a().getAsBoolean() ? 0.0 : 1.0);
                double y = -joystick.getLeftX() * MaxSpeed * 0.25 * (extraDebugJoystick.a().getAsBoolean() ? 0.0 : 1.0);
                x += -extraDebugJoystick.getLeftY() * MaxSpeed;
                y += -extraDebugJoystick.getLeftX() * MaxSpeed;
                double rot = rotation;
                    return drive.withVelocityX(x) // Drive forward with negative Y (forward)
                        .withVelocityY(y) // Drive left with negative X (left)
                        .withRotationalRate(rot); // Drive counterclockwise with negative X (left)\
                }
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
            feeder.getDutyCycleCommand(() -> 1.0, () -> -1.0)
        );
    }

    public Command shootFixedVelocityCommand() {
        return Commands.parallel(
            flywheels.velocity(() -> Rotation2d.fromRotations(flywheels.getVelocityForDistance(4))),
            feeder.getDutyCycleCommand(() -> 1.0, () -> -1.0).beforeStarting(new WaitCommand(1.5)),
            ((intake.holdStateWithActiveRollers(0.0)
                .beforeStarting(intake.setState(IntakePivotState.DOWN)))
                    .withTimeout(0.75).andThen(
                       (intake.holdStateWithActiveRollers(-0.65)
                .beforeStarting(intake.setState(IntakePivotState.UP)))
                    .withTimeout(0.4)).repeatedly())
        );
    }

    public Command shootFixedVelocityBigCommand() {
        return Commands.parallel(
            flywheels.velocity(() -> Rotation2d.fromRotations(flywheels.getVelocityForDistance(40))),
            feeder.getDutyCycleCommand(() -> 1.0, () -> -1.0).beforeStarting(new WaitCommand(1.5)),
            ((intake.holdStateWithActiveRollers(0.0)
                .beforeStarting(intake.setState(IntakePivotState.DOWN)))
                    .withTimeout(0.75).andThen(
                       (intake.holdStateWithActiveRollers(-0.65)
                .beforeStarting(intake.setState(IntakePivotState.UP)))
                    .withTimeout(0.4)).repeatedly())
        );
    }

    public Command autoShootCommand() {
        return Commands.parallel(
            flywheels.velocityFor(CustomMath.makeTranslationAllianceRelative(FieldConstants.Hub.innerCenterPoint.toTranslation2d())),
            feeder.getDutyCycleCommand(() -> 1.0, () -> -1.0),
            intake.holdStateWithActiveRollers(0.0)
                    .beforeStarting(intake.setState(IntakePivotState.UP)).beforeStarting(new WaitCommand(2))
        );
    }

    public Command getAutonomousCommand() {
        return new WaitCommand(SmartDashboard.getNumber("Autonomous Starting Delay", 0.0)).andThen(autoChooser.getSelected().asProxy());
    }
}