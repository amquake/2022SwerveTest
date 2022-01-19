package frc.robot.common;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Conversions;

public class SwerveModule {

    private final Constants.Swerve.Module moduleConstants;
    private double lastTargetTotalAngle = 0;

    // Hardware
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX steerMotor;
    private final WPI_CANCoder steerEncoder;

    // Linear drive feed forward
    public final SimpleMotorFeedforward driveFF = RobotBase.isReal() ? Constants.Swerve.kDriveFF : Constants.Sim.kDriveFF;
    // Steer feed forward
    public final SimpleMotorFeedforward steerFF = RobotBase.isReal() ? Constants.Swerve.kSteerFF : Constants.Sim.kSteerFF;

    public SwerveModule(Constants.Swerve.Module moduleConstants, CTREConfigs configs){
        this.moduleConstants = moduleConstants;

        driveMotor = new WPI_TalonFX(moduleConstants.driveMotorID);
        steerMotor = new WPI_TalonFX(moduleConstants.steerMotorID);
        steerEncoder = new WPI_CANCoder(moduleConstants.cancoderID);

        setupDriveMotor(configs);
        setupCancoder(configs);
        setupSteerMotor(configs);

        // Simulation
        driveMotorSim = driveMotor.getSimCollection();
        steerMotorSim = steerMotor.getSimCollection();
        steerEncoderSim = steerEncoder.getSimCollection();
    }

    private void setupDriveMotor(CTREConfigs configs){
        driveMotor.configAllSettings(configs.swerveDriveConfig);
        setupDriveMotor();
    }
    private void setupDriveMotor(){
        driveMotor.enableVoltageCompensation(true);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setSelectedSensorPosition(0);
        driveMotor.setInverted(Constants.Swerve.kInvertDrive);
    }
    private void setupCancoder(CTREConfigs configs){
        steerEncoder.configAllSettings(configs.swerveCancoderConfig);
        steerEncoder.configMagnetOffset(moduleConstants.angleOffset, 30);
    }
    private void setupSteerMotor(CTREConfigs configs){
        steerMotor.configAllSettings(configs.swerveSteerConfig);
        setupSteerMotor();
    }
    private void setupSteerMotor(){
        steerMotor.enableVoltageCompensation(true);
        steerMotor.setNeutralMode(NeutralMode.Brake);
        steerMotor.setInverted(Constants.Swerve.kInvertSteer);
        resetToAbsolute();
    }

    public void periodic(){
        // check if the motors had an oopsie, reapply settings
        if(driveMotor.hasResetOccurred()){
            setupDriveMotor();
        }
        if(steerMotor.hasResetOccurred()){
            setupSteerMotor();
        }
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
     * @param steerInPlace If modules should steer to target angle when target velocity is 0
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean steerInPlace){
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
        if(!steerInPlace && Math.abs(desiredState.speedMetersPerSecond) < 0.05){
            targetTotalAngle = lastTargetTotalAngle;
        }
        else{
            lastTargetTotalAngle = targetTotalAngle;
        }

        // convert our target radians to falcon position units
        double angle = Conversions.degreesToFalcon(Math.toDegrees(targetTotalAngle), Constants.Swerve.kSteerGearRatio);
        // perform onboard PID to steer the module to the target angle
        steerMotor.set(ControlMode.Position, angle);
        //steerMotor.set(ControlMode.MotionMagic, angle);

        // convert our target meters per second to falcon velocity units
        SmartDashboard.putNumber("Module "+moduleConstants.moduleNum+" Target Velocity Feet", Units.metersToFeet(desiredState.speedMetersPerSecond));
        double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.kWheelCircumference, Constants.Swerve.kDriveGearRatio);
        // perform onboard PID with inputted feedforward to drive the module to the target velocity
        driveMotor.set(
            ControlMode.Velocity, velocity, // Native falcon counts per 100ms
            DemandType.ArbitraryFeedForward, driveFF.calculate(desiredState.speedMetersPerSecond)/Constants.Swerve.kVoltageSaturation // feedforward voltage to percent output
        );
    }

    public void setDriveBrake(boolean is){
        driveMotor.setNeutralMode(is ? NeutralMode.Brake : NeutralMode.Coast);
    }
    public void setSteerBrake(boolean is){
        steerMotor.setNeutralMode(is ? NeutralMode.Brake : NeutralMode.Coast);
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

    /**
     * @return Constants about this module, like motor IDs, cancoder angle offset, and translation from center
     */
    public Constants.Swerve.Module getModuleConstants(){
        return moduleConstants;
    }

    public void log(){
        SwerveModuleState state = getState();
        int num = moduleConstants.moduleNum;
        SmartDashboard.putNumber("Module "+num+" Cancoder Degrees", getCancoderHeading().getDegrees());
        SmartDashboard.putNumber("Module "+num+" Steer Degrees", state.angle.getDegrees());
        SmartDashboard.putNumber("Module "+num+" Steer Velocity", steerMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Module "+num+" Velocity Feet", Units.metersToFeet(state.speedMetersPerSecond));
    }


    // Simulation
    private final TalonFXSimCollection driveMotorSim;
    private final FlywheelSim driveMotorSimModel = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            driveFF.kv * Constants.Swerve.kWheelCircumference / (2*Math.PI),
            driveFF.ka * Constants.Swerve.kWheelCircumference / (2*Math.PI)
        ),
        DCMotor.getFalcon500(1),
        Constants.Swerve.kDriveGearRatio
    );
    private final TalonFXSimCollection steerMotorSim;
    private final FlywheelSim steerMotorSimModel = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(steerFF.kv, steerFF.ka),
        DCMotor.getFalcon500(1),
        Constants.Swerve.kSteerGearRatio
    );
    private final CANCoderSimCollection steerEncoderSim;

    public void simulationPeriodic(){
        driveMotorSimModel.setInputVoltage(driveMotor.getMotorOutputVoltage());
        steerMotorSimModel.setInputVoltage(steerMotor.getMotorOutputVoltage());
        driveMotorSimModel.update(0.02);
        steerMotorSimModel.update(0.02);

        SmartDashboard.putNumber("Drive Sim Model Amps", driveMotorSimModel.getCurrentDrawAmps());
        SmartDashboard.putNumber("Drive Sim Model Velocity Feet", Units.metersToFeet(driveMotorSimModel.getAngularVelocityRPM() * Constants.Swerve.kWheelCircumference / 60));
        double driveMotorVelocityNative = driveMotorSimModel.getAngularVelocityRPM() * 2048 / 600 * Constants.Swerve.kDriveGearRatio;
        double driveMotorPositionDeltaNative = driveMotorVelocityNative*10*0.02;
        driveMotorSim.setIntegratedSensorVelocity((int)driveMotorVelocityNative);
        driveMotorSim.addIntegratedSensorPosition((int)(driveMotorPositionDeltaNative));

        SmartDashboard.putNumber("Steer Sim Model Velocity", steerMotorSimModel.getAngularVelocityRPM());
        double steerMotorVelocityNative = steerMotorSimModel.getAngularVelocityRPM() * 2048 / 600 * Constants.Swerve.kSteerGearRatio;
        double steerMotorPositionDeltaNative = steerMotorVelocityNative*10*0.02;
        steerMotorSim.setIntegratedSensorVelocity((int)steerMotorVelocityNative);
        steerMotorSim.addIntegratedSensorPosition((int)(steerMotorPositionDeltaNative));
        
        steerEncoderSim.setVelocity((int)steerMotorVelocityNative);
        steerEncoderSim.addPosition((int)(steerMotorPositionDeltaNative));

        driveMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        steerMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        steerEncoderSim.setBusVoltage(RobotController.getBatteryVoltage());
    }
}
