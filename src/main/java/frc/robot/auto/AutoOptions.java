package frc.robot.auto;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.OCSwerveFollower;
import frc.robot.subsystems.Drivetrain;

public class AutoOptions {
    
    private final Field2d field2d;
    private SendableChooser<Command> autoOptions = new SendableChooser<>();

    public AutoOptions(Drivetrain drivetrain, Field2d field2d){

        this.field2d = field2d;

        autoOptions.setDefaultOption("Nothing",
            new InstantCommand(()->drivetrain.drive(0, 0, 0, false), drivetrain)
        );

        autoOptions.addOption("Ellipse",
            autoFollowTrajectories(
                drivetrain,
                TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                        new Pose2d(2, 2, Rotation2d.fromDegrees(90)),
                        new Pose2d(4, 2, Rotation2d.fromDegrees(-90)),
                        new Pose2d(2, 2, Rotation2d.fromDegrees(90))
                    ),
                    new TrajectoryConfig(Units.feetToMeters(6), Units.feetToMeters(8))
                )
            )
        );

        // 2022 auto mockup
        autoOptions.addOption("Mockup",
            autoFollowTrajectories(
                drivetrain,
                TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                        new Pose2d(7.5, 2.9, Rotation2d.fromDegrees(70)),
                        new Pose2d(7.3, 0.7, Rotation2d.fromDegrees(90))
                    ),
                    new TrajectoryConfig(Units.feetToMeters(6), Units.feetToMeters(8)).setReversed(true)
                ),
                TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                        new Pose2d(7.3, 0.7, Rotation2d.fromDegrees(90)),
                        new Pose2d(5.3, 1.85, Rotation2d.fromDegrees(180))
                    ),
                    new TrajectoryConfig(Units.feetToMeters(6), Units.feetToMeters(8))
                )
            )
        );
    }

    private Command autoFollowTrajectories(Drivetrain drivetrain, Trajectory... trajectories){
        if(trajectories.length==0) return new InstantCommand(()->{}, drivetrain);
        Command sequence = new InstantCommand(()->{
            drivetrain.resetOdometry(trajectories[0].getInitialPose());
        });
        for(Trajectory trajectory : trajectories){
            sequence = sequence.andThen(
                new OCSwerveFollower(drivetrain, trajectory)
                    .beforeStarting(()->submitTrajectory(trajectory))
            );
        }
        return sequence;
    }


    // Network Tables
    public Command getSelected(){
        return autoOptions.getSelected();
    }

    public void submitTrajectory(Trajectory trajectory){
        field2d.getObject("Trajectory").setTrajectory(trajectory);
    }
    public void submitTrajectory(Pose2d... poses){
        field2d.getObject("Trajectory").setPoses(poses);;
    }

    public void submit(){
        SmartDashboard.putData("Auto Options", autoOptions);
    }
}
