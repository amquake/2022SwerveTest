package frc.robot.common;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Conversions;

public class SwerveModule {

    private final double angleOffset;
    private double lastTargetTotalAngle = 0;

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

        driveMotor.configAllSettings(configs.swerveDriveConfig);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setSelectedSensorPosition(0);
        driveMotor.setInverted(Constants.Swerve.kInvertDrive);

        steerEncoder.configAllSettings(configs.swerveCancoderConfig);
        steerEncoder.configMagnetOffset(angleOffset, 30);

        steerMotor.configAllSettings(configs.swerveSteerConfig);
        steerMotor.setNeutralMode(NeutralMode.Brake);
        steerMotor.setInverted(Constants.Swerve.kInvertSteer);
        resetToAbsolute();
    }

    /**
     * Reset the steering motor integrated encoder to the position of the steering cancoder.
     * We want to use the integrated encoder for control, but need the absolute cancoder for determining our startup rotation.
     */
    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCancoderHeading().getDegrees(), Constants.Swerve.kSteerGearRatio);
        steerMotor.setSelectedSensorPosition(absolutePosition);
    }

    /**
     * Command this swerve module to the desired angle and velocity.
     * Falcon onboard control is used instead of on-RIO control to avoid latency.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean spinInPlace){
        Rotation2d currentRotation = getIntegratedHeading();
        desiredState = SwerveModuleState.optimize(desiredState, currentRotation);
        
        // our desired angle in [-pi, pi]
        double targetConstrainedAngle = desiredState.angle.getRadians();
        // our total current angle. This is not constrained to [-pi, pi]
        double currentTotalAngle = currentRotation.getRadians();
        // our current angle in [-pi, pi]
        double currentConstrainedAngle = MathUtil.angleModulus(currentTotalAngle);
        // convert our constrained target to the closest "total" angle near our current total
        double targetTotalAngle = currentTotalAngle + MathUtil.angleModulus(targetConstrainedAngle - currentConstrainedAngle);

        // if the module is not driving, maintain last angle setpoint
        if(!spinInPlace && Math.abs(desiredState.speedMetersPerSecond) < 0.05){
            targetTotalAngle = lastTargetTotalAngle;
        }
        else{
            lastTargetTotalAngle = targetTotalAngle;
        }

        // convert our target radians to falcon position units
        double angle = Conversions.degreesToFalcon(Math.toDegrees(targetTotalAngle), Constants.Swerve.kSteerGearRatio);
        // perform onboard PID to steer the module to the target angle
        steerMotor.set(ControlMode.Position, angle);

        // convert our target meters per second to falcon velocity units
        double velocity = Conversions.falconToMPS(desiredState.speedMetersPerSecond, Constants.Swerve.kWheelCircumference, Constants.Swerve.kDriveGearRatio);
        // perform onboard PID with inputted feedforward to drive the module to the target velocity
        driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, driveFF.calculate(desiredState.speedMetersPerSecond));
    }

    public void setDriveBrake(boolean is){
        driveMotor.setNeutralMode(is ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Module heading reported by steering motor integrated encoder.
     * <br></br>
     * NOT constrained to [-pi, pi]
     */
    public Rotation2d getIntegratedHeading(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(), Constants.Swerve.kSteerGearRatio));
    }
    /**
     * Module heading reported by steering cancoder
     */
    public Rotation2d getCancoderHeading(){
        return Rotation2d.fromDegrees(steerEncoder.getAbsolutePosition());
    }

    /**
     * @return State describing module rotation(heading) and velocity in meters per second
     */
    public SwerveModuleState getState(){
        double velocity = Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), Constants.Swerve.kWheelCircumference, Constants.Swerve.kDriveGearRatio);
        Rotation2d angle = getIntegratedHeading();
        return new SwerveModuleState(velocity, angle);
    }

    public void log(int moduleNum){
        SwerveModuleState state = getState();
        SmartDashboard.putNumber("Module "+moduleNum+" Cancoder Degrees", getCancoderHeading().getDegrees());
        SmartDashboard.putNumber("Module "+moduleNum+" Steer Degrees", state.angle.getDegrees());
        SmartDashboard.putNumber("Module "+moduleNum+" Velocity Feet", Units.metersToFeet(state.speedMetersPerSecond));
    }
}
