package frc.robot.common;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public class SwerveModule {

    private final double angleOffset;

    // Hardware
    private WPI_TalonFX driveMotor;
    private WPI_TalonFX steerMotor;
    private WPI_CANCoder steerEncoder;

    // Linear drive feed forward
    public final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(
        Constants.Swerve.kDriveStaticFF, // Voltage to break static friction
        Constants.Swerve.kDriveVelocityFF, // Volts per meter per second
        Constants.Swerve.kDriveAccelFF // Volts per meter per second squared
    );
    // Steer feed forward
    public final SimpleMotorFeedforward steerFF = new SimpleMotorFeedforward(
        Constants.Swerve.kSteerStaticFF, // Voltage to break static friction
        Constants.Swerve.kSteerVelocityFF, // Volts per meter per second
        Constants.Swerve.kSteerAccelFF // Volts per meter per second squared
    );

    public SwerveModule(Constants.Swerve.Module moduleConstants, CTREConfigs configs){
        this.angleOffset = moduleConstants.angleOffset;

        driveMotor = new WPI_TalonFX(moduleConstants.driveMotorID);
        steerMotor = new WPI_TalonFX(moduleConstants.steerMotorID);
        steerEncoder = new WPI_CANCoder(moduleConstants.cancoderID);

        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(configs.swerveDriveConfig);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setSelectedSensorPosition(0);

        steerMotor.configFactoryDefault();
        steerMotor.configAllSettings(configs.swerveSteerConfig);
        steerMotor.setNeutralMode(NeutralMode.Brake);
        steerMotor.setSelectedSensorPosition(0);
    }
}
