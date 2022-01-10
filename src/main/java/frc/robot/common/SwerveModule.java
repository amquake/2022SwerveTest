package frc.robot.common;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public class SwerveModule {

    /* Constants */
    private static final double kWheelDiameter = Units.inchesToMeters(4);

    private static final double kDriveGearRatio = 6.75; // 6.75:1
    private static final double kSteerGearRatio = 12.8; // 12.8:1

    // Current limits
    private static final int driveContinuousCurrentLimit = 38;
    private static final int drivePeakCurrentLimit = 65;
    private static final double drivePeakCurrentDuration = 0.15;

    private static final int steerContinuousCurrentLimit = 25;
    private static final int steerPeakCurrentLimit = 40;
    private static final double steerPeakCurrentDuration = 0.1;

    // PID
    private static final double driveKP = 0;
    private static final double driveKI = 0;
    private static final double driveKD = 0;

    private static final double steerKP = 0;
    private static final double steerKI = 0;
    private static final double steerKD = 0;

    private final int driveMotorID;
    private final int steerMotorID;
    private final int cancoderID;
    private final double angleOffset;

    // Hardware
    private WPI_TalonFX driveMotor;
    private WPI_TalonFX steerMotor;
    private WPI_CANCoder steerEncoder;

    // Control
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        0, // Voltage to break static friction
        0, // Volts per meter per second
        0 // Volts per meter per second squared
    );

    public SwerveModule(int driveMotorID, int steerMotorID, int cancoderID, double angleOffset){
        this.driveMotorID = driveMotorID;
        this.steerMotorID = steerMotorID;
        this.cancoderID = cancoderID;
        this.angleOffset = angleOffset;

        driveMotor = new WPI_TalonFX(driveMotorID);
        steerMotor = new WPI_TalonFX(steerMotorID);
        steerEncoder = new WPI_CANCoder(cancoderID);
    }
}
