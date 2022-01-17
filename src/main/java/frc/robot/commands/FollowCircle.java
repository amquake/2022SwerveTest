// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.Constants;
import frc.robot.subsystems.Drivetrain;

public class FollowCircle extends CommandBase {
    
    private final Drivetrain drivetrain;

    private final Pose2d initialPose;
    private final double diameterMeters;
    private final Pose2d circleCenter;
    private final double kCircumference;
    private final TrapezoidProfile profile;

    private Timer timer = new Timer();

    public FollowCircle(Drivetrain drivetrain, double diameterMeters, Rotation2d robotToCircleCenter, TrajectoryConfig config) {
        this.drivetrain = drivetrain;
        this.initialPose = drivetrain.getPose();
        this.diameterMeters = diameterMeters;
        this.circleCenter = initialPose.transformBy(new Transform2d(
            new Translation2d(diameterMeters/2, robotToCircleCenter), // circle center on field
            robotToCircleCenter.plus(new Rotation2d(Math.PI))) // circle angle to initial robot pose
        );

        this.kCircumference = Math.PI * diameterMeters;
        this.profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(config.getMaxVelocity(), config.getMaxAcceleration()),
            new TrapezoidProfile.State(kCircumference, config.getEndVelocity()),
            new TrapezoidProfile.State(0, config.getStartVelocity())
        );

        addRequirements(drivetrain);
    }
        
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // position and velocity along the circle at current time
        TrapezoidProfile.State targetState = profile.calculate(timer.get());
        // parametrize target arc length to [0, 2pi] to get position along circle
        double t = targetState.position / kCircumference * 2 * Math.PI;
        // angle from circle center to the target pose
        Rotation2d centerToTarget = new Rotation2d(t).plus(circleCenter.getRotation());
        Pose2d targetPose = new Pose2d(
            new Translation2d(diameterMeters/2, centerToTarget).plus(circleCenter.getTranslation()),
            centerToTarget.plus(new Rotation2d(Math.PI/2)) // tangent angle
        );

        // determine ChassisSpeeds from tangent velocity and positional feedback control from HolonomicDriveController
        ChassisSpeeds targetChassisSpeeds = drivetrain.getPathController().calculate(
            drivetrain.getPose(),
            targetPose,
            targetState.velocity,
            centerToTarget.rotateBy(new Rotation2d(Math.PI)) // point swerve at circle center
        );

        // command robot to reach the target ChassisSpeeds
        drivetrain.setModuleStates(
            Constants.Swerve.kinematics.toSwerveModuleStates(targetChassisSpeeds),
            false
        );
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return profile.isFinished(timer.get());
    }
}
