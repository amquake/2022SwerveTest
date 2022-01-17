package frc.robot.auto;

import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.OCSwerveFollower;
import frc.robot.subsystems.Drivetrain;

public class AutoOptions {
    
    SendableChooser<Command> autoOptions = new SendableChooser<>();

    public AutoOptions(Drivetrain drivetrain){

        autoOptions.setDefaultOption("Nothing",
            new InstantCommand(()->drivetrain.drive(0, 0, 0, false), drivetrain)
        );

        // drive forward in a 3ft circle(ellipse)
        autoOptions.addOption("Circle",
            new OCSwerveFollower(
                drivetrain,
                TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, Rotation2d.fromDegrees(-90)),
                    List.of(new Translation2d(1,0)),
                    new Pose2d(0, 0, Rotation2d.fromDegrees(-90)),
                    new TrajectoryConfig(Units.feetToMeters(6), Units.feetToMeters(8))
                )
            )
        );
    }

    public Command getSelected(){
        return autoOptions.getSelected();
    }

    public void submit(){
        SmartDashboard.putData("Auto Options", autoOptions);
    }
}
