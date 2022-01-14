package frc.robot.common;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Conversions;

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

        steerEncoder.configFactoryDefault();
        steerEncoder.configAllSettings(configs.swerveCancoderConfig);

        steerMotor.configFactoryDefault();
        steerMotor.configAllSettings(configs.swerveSteerConfig);
        steerMotor.setNeutralMode(NeutralMode.Brake);
        resetToAbsolute();
    }

    public Rotation2d getCancoder(){
        return Rotation2d.fromDegrees(steerEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState(){
        double velocity = Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), Constants.Swerve.kWheelCircumference, Constants.Swerve.kDriveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(), Constants.Swerve.kSteerGearRatio));
        return new SwerveModuleState(velocity, angle);
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCancoder().getDegrees() - angleOffset, Constants.Swerve.kSteerGearRatio);
        steerMotor.setSelectedSensorPosition(absolutePosition);
    }

    public void setDesiredState(SwerveModuleState desiredState){
        
        double angle = desiredState.angle.getDegrees();
        steerMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angleOffset, Constants.Swerve.kSteerGearRatio));
        double velocity = Conversions.falconToMPS(desiredState.speedMetersPerSecond, Constants.Swerve.kWheelCircumference, Constants.Swerve.kDriveGearRatio);
        driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, driveFF.calculate(desiredState.speedMetersPerSecond));
    }
}
