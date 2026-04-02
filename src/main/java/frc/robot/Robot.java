// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.Turret;
import frc.robot.util.AlertContainer;
import frc.robot.util.CustomMath;
import frc.robot.util.HubTracker;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    public final RobotContainer m_robotContainer;
    private final StructPublisher<Pose2d> visionPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Vision Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Target Pose", Pose2d.struct).publish();


    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        SmartDashboard.putNumber("Autonomous Starting Delay", 0);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        Drivetrain drivetrain = m_robotContainer.drivetrain;
        SmartDashboard.putNumber("Distance from Goal", drivetrain.getStateCopy().Pose.getTranslation().getDistance(FieldConstants.Hub.innerCenterPoint.toTranslation2d()));
        SmartDashboard.putNumber("Pose Heading", drivetrain.getStateCopy().Pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        HubTracker.timeRemainingInCurrentShift().ifPresentOrElse((time) -> {
            SmartDashboard.putNumber("Seconds Remaining In Shift", Math.round(time.in(Seconds)));
        }, () -> SmartDashboard.putNumber("Seconds Remaining In Shift", 0));
        SmartDashboard.putBoolean("Shift Active", HubTracker.isActive());
        visionPosePublisher.set(Localization.getInstance().getVisionMT2Pose().pose);
        targetPosePublisher.set(CustomMath.makePoseAllianceRelative(new Pose2d(Turret.getInstance().currentLandmark, Rotation2d.kZero)));
        drivetrain.updateField();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
            // If we have invalid game data, assume hub is active.
            return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
        }
}
