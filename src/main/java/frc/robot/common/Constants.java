package frc.robot.common;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class Swerve {

        public static final int kPigeonID = 0;

        // Inversions
        public static final boolean kInvertGyro = false;
        public static final boolean kInvertDrive = false;
        public static final boolean kInvertSteer = false;
        public static final boolean kInvertCancoder = false;

        // Physical properties
        public static final double kTrackWidth = Units.inchesToMeters(23);
        public static final double kTrackLength = Units.inchesToMeters(23);
        
        public static final double kMaxLinearSpeed = Units.feetToMeters(14);
        public static final double kMaxAngularSpeed = Units.degreesToRadians(720);
        public static final double kWheelDiameter = Units.inchesToMeters(4);
        public static final double kWheelCircumference = kWheelDiameter*Math.PI;
        public static final double kDriveGearRatio = 6.75; // 6.75:1
        public static final double kSteerGearRatio = 12.8; // 12.8:1

        public enum Module {
            FL(1, 0, 0, 0, 0, kTrackLength/2, kTrackWidth/2), // Front left
            FR(2, 0, 0, 0, 0, kTrackLength/2, -kTrackWidth/2),
            BL(3, 0, 0, 0, 0, -kTrackLength/2, kTrackWidth/2),
            BR(4, 0, 0, 0, 0, -kTrackLength/2, -kTrackWidth/2);

            public final int moduleNum;
            public final int driveMotorID;
            public final int steerMotorID;
            public final int cancoderID;
            public final double angleOffset;
            public final Translation2d centerOffset;
            private Module(int moduleNum, int driveMotorID, int steerMotorID, int cancoderID, double angleOffset, double xOffset, double yOffset){
                this.moduleNum = moduleNum;
                this.driveMotorID = driveMotorID;
                this.steerMotorID = steerMotorID;
                this.cancoderID = cancoderID;
                this.angleOffset = angleOffset;
                centerOffset = new Translation2d(xOffset, yOffset);
            }
        }

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            Module.FL.centerOffset,
            Module.FR.centerOffset,
            Module.BL.centerOffset,
            Module.BR.centerOffset
        );

        // Current limits
        public static final int kDriveContinuousCurrentLimit = 40;
        public static final int kDrivePeakCurrentLimit = 65;
        public static final double kDrivePeakCurrentDuration = 0.1;

        public static final int kSteerContinuousCurrentLimit = 25;
        public static final int kSteerPeakCurrentLimit = 40;
        public static final double kSteerPeakCurrentDuration = 0.1;

        // Feedforward
        public static final double kDriveStaticFF = 0; // Voltage to break static friction
        public static final double kDriveVelocityFF = 0; // Volts per meter per second
        public static final double kDriveAccelFF = 0; // Volts per meter per second squared

        public static final double kSteerStaticFF = 0; // Voltage to break static friction
        public static final double kSteerVelocityFF = 0; // Volts per meter per second
        public static final double kSteerAccelFF = 0; // Volts per meter per second squared

        // PID
        public static final double kDriveKP = 0;
        public static final double kDriveKI = 0;
        public static final double kDriveKD = 0;

        public static final double kSteerKP = 0;
        public static final double kSteerKI = 0;
        public static final double kSteerKD = 0;
    }

    public static final class Auto {
        // our maximum speeds/accelerations during auto
        public static final double kMaxLinearSpeed = Units.feetToMeters(8);
        public static final double kMaxLinearAcceleration = Units.feetToMeters(10);
        public static final double kMaxAngularSpeed = Units.degreesToRadians(540);
        public static final double kMaxAngularAcceleration = Units.degreesToRadians(540);

        public static final double kPXController = 1; // pose PID control. 1 meter error in x = 1 meter per second x velocity 
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
        // constraints for the theta controller on velocity (omega) and acceleration (omega squared)
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeed,
            kMaxAngularAcceleration
        );

        // packaged configs for path following
        public static final TrajectoryConfig kMaxSpeedConfig = new TrajectoryConfig(kMaxLinearSpeed, kMaxLinearAcceleration);
        public static final TrajectoryConfig kMediumSpeedConfig = new TrajectoryConfig(0.6*kMaxLinearSpeed, 0.6*kMaxLinearAcceleration);
        public static final TrajectoryConfig kSlowSpeedConfig = new TrajectoryConfig(0.3*kMaxLinearSpeed, 0.3*kMaxLinearAcceleration);
    }
}
