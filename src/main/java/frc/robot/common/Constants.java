package frc.robot.common;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class Swerve {

        public static final double kWheelDiameter = Units.inchesToMeters(4);
        public static final double kDriveGearRatio = 6.75; // 6.75:1
        public static final double kSteerGearRatio = 12.8; // 12.8:1

        // Current limits
        public static final int driveContinuousCurrentLimit = 38;
        public static final int drivePeakCurrentLimit = 65;
        public static final double drivePeakCurrentDuration = 0.15;

        public static final int steerContinuousCurrentLimit = 25;
        public static final int steerPeakCurrentLimit = 40;
        public static final double steerPeakCurrentDuration = 0.1;

        // Linear drive feed forward
        public final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(
            0, // Voltage to break static friction
            0, // Volts per meter per second
            0 // Volts per meter per second squared
        );
        // Steer feed forward
        public final SimpleMotorFeedforward steerFF = new SimpleMotorFeedforward(
            0, // Voltage to break static friction
            0, // Volts per meter per second
            0 // Volts per meter per second squared
        );

        // PID
        public static final double driveKP = 0;
        public static final double driveKI = 0;
        public static final double driveKD = 0;

        public static final double steerKP = 0;
        public static final double steerKI = 0;
        public static final double steerKD = 0;

        public enum Module {
            FL(0, 0, 0, 0), // Front left
            FR(0, 0, 0, 0),
            BL(0, 0, 0, 0),
            BR(0, 0, 0, 0);

            public final int driveMotorID;
            public final int steerMotorID;
            public final int cancoderID;
            public final double angleOffset;
            private Module(int driveMotorID, int steerMotorID, int cancoderID, double angleOffset){
                this.driveMotorID = driveMotorID;
                this.steerMotorID = steerMotorID;
                this.cancoderID = cancoderID;
                this.angleOffset = angleOffset;
            }
        }
    }
}
