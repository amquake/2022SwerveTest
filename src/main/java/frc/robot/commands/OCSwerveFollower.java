// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.Constants;
import frc.robot.subsystems.Drivetrain;

public class OCSwerveFollower extends CommandBase {

    private final Drivetrain drivetrain;
    private final Trajectory trajectory;
    private Timer timer = new Timer();

    public OCSwerveFollower(Drivetrain drivetrain, Trajectory trajectory) {
        this.drivetrain = drivetrain;
        this.trajectory = trajectory;

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
        double currentTime = timer.get();
        // The target state of the trajectory right now (the robot's pose and velocity)
        Trajectory.State targetState = trajectory.sample(currentTime);

        // determine ChassisSpeeds from path state and feedback control from HolonomicDriveController
        ChassisSpeeds targetChassisSpeeds = drivetrain.getPathController().calculate(
            drivetrain.getPose(),
            targetState,
            targetState.poseMeters.getRotation()
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
        // the path is time parametrized and takes a certain number of seconds
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}
