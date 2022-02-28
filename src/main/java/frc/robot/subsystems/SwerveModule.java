package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.constants.SwerveConstants.*;

import frc.robot.constants.SwerveConstants;
import frc.robot.util.TalonUtil;

public class SwerveModule {

    private final SwerveConstants.Module moduleConstants;
    private SwerveModuleState lastDesiredState = new SwerveModuleState();

    // Hardware
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX steerMotor;
    private final WPI_CANCoder steerEncoder;

    // Steer controller
    private final ProfiledPIDController steerController = new ProfiledPIDController(
        kSteerKP, kSteerKI, kSteerKD,
        kSteerConstraints
    );

    public SwerveModule(SwerveConstants.Module moduleConstants){
        this.moduleConstants = moduleConstants;

        driveMotor = new WPI_TalonFX(moduleConstants.driveMotorID);
        steerMotor = new WPI_TalonFX(moduleConstants.steerMotorID);
        steerEncoder = new WPI_CANCoder(moduleConstants.cancoderID);

        setupDriveMotor(true);
        setupCancoder(true);
        setupSteerMotor(true);

        steerController.enableContinuousInput(-Math.PI, Math.PI);

        // Simulation
        driveMotorSim = driveMotor.getSimCollection();
        steerMotorSim = steerMotor.getSimCollection();
        steerEncoderSim = steerEncoder.getSimCollection();
    }

    private void setupDriveMotor(boolean init){
        if(init){
            driveMotor.configAllSettings(driveConfig, kCANTimeout);
        }
        driveMotor.enableVoltageCompensation(true);
        driveMotor.setSelectedSensorPosition(0);
        driveMotor.setInverted(kInvertDrive);
        TalonUtil.configStatusNormal(driveMotor);
    }
    private void setupCancoder(boolean init){
        steerEncoder.configAllSettings(cancoderConfig, kCANTimeout);
        steerEncoder.configMagnetOffset(moduleConstants.angleOffset, kCANTimeout);
    }
    private void setupSteerMotor(boolean init){
        if(init){
            steerMotor.configAllSettings(steerConfig);
        }
        steerMotor.enableVoltageCompensation(true);
        steerMotor.setInverted(kInvertSteer);
        TalonUtil.configStatusNormal(steerMotor);
    }

    public void periodic(){
        // check if the motors had an oopsie, reapply settings
        if(driveMotor.hasResetOccurred()){
            setupDriveMotor(false);
        }
        if(steerMotor.hasResetOccurred()){
            setupSteerMotor(false);
        }
    }

    /**
     * Command this swerve module to the desired angle and velocity.
     * Falcon onboard control is used instead of on-RIO control to avoid latency.
     * @param steerInPlace If modules should steer to target angle when target velocity is 0
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean openLoop, boolean steerInPlace){
        Rotation2d currentRotation = getAbsoluteHeading();
        desiredState = SwerveModuleState.optimize(desiredState, currentRotation);
        
        // if the module is not driving, maintain last angle setpoint
        if(!steerInPlace && Math.abs(desiredState.speedMetersPerSecond) < 0.01){
            desiredState.angle = lastDesiredState.angle;
        }

        // calculate our next profile setpoint and subsequent PID output on setpoint position
        double steerPIDOutput = steerController.calculate(getAbsoluteHeading().getRadians(), desiredState.angle.getRadians());
        if(RobotBase.isSimulation()){
            // promote small PID outputs to get better control close to the setpoint
            steerPIDOutput += Math.signum(steerPIDOutput) * kSteerFF.ks*0.95;
        }
        // calculate the feedforward voltage to achieve setpoint velocity
        double steerFFOutput = kSteerFF.calculate(steerController.getSetpoint().velocity);
        SmartDashboard.putNumber("Module "+moduleConstants.moduleNum+"/Steer PID", steerPIDOutput*100);
        //double steerFFOutput = kSteerFF.calculate(steerController.getSetpoint().velocity);
        // apply our calculated voltage to the steering motor
        steerMotor.set(
            ControlMode.PercentOutput,
            (steerFFOutput + steerPIDOutput)/kVoltageSaturation
        );

        // convert our target meters per second to falcon velocity units
        double velocityNative = TalonUtil.metersToVelocity(
            desiredState.speedMetersPerSecond,
            kDriveGearRatio,
            kWheelCircumference
        );
        double driveFFOutput = kDriveFF.calculate(desiredState.speedMetersPerSecond);
        if(openLoop){
            // drive open-loop (without PID) for driver preference
            driveMotor.set(ControlMode.PercentOutput, driveFFOutput/kVoltageSaturation);
        }
        else{
            // perform onboard PID with inputted feedforward to drive the module to the target velocity
            driveMotor.set(
                ControlMode.Velocity, velocityNative, // Native falcon counts per 100ms
                DemandType.ArbitraryFeedForward, driveFFOutput/kVoltageSaturation // feedforward voltage to percent output
            );
        }

        lastDesiredState = desiredState;
    }

    public void setSteerVelocityRadians(double velocityRadians){
        double steerFFOutput = kSteerFF.calculate(velocityRadians);
        // apply our calculated voltage to the steering motor
        steerMotor.set(
            ControlMode.PercentOutput,
            (steerFFOutput)/kVoltageSaturation
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
        return Rotation2d.fromDegrees(TalonUtil.positionToDegrees(steerMotor.getSelectedSensorPosition(), kSteerGearRatio));
    }
    /**
     * Module heading reported by steering cancoder [-pi, pi]
     */
    public Rotation2d getAbsoluteHeading(){
        return Rotation2d.fromDegrees(steerEncoder.getAbsolutePosition());
    }

    /**
     * @return State describing integrated module rotation and velocity in meters per second
     */
    public SwerveModuleState getIntegratedState(){
        double velocity = TalonUtil.velocityToMeters(
            driveMotor.getSelectedSensorVelocity(),
            kDriveGearRatio, kWheelCircumference
        );
        Rotation2d angle = getIntegratedHeading();
        return new SwerveModuleState(velocity, angle);
    }
    /**
     * @return Radians per second of the steering cancoder (NOT the integrated encoder)
     */
    public double getSteerVelocityRadians(){
        return Units.degreesToRadians(steerEncoder.getVelocity());
    }
    /**
     * @return State describing absolute module rotation and velocity in meters per second
     */
    public SwerveModuleState getAbsoluteState(){
        double velocity = TalonUtil.velocityToMeters(
            driveMotor.getSelectedSensorVelocity(),
            kDriveGearRatio, kWheelCircumference
        );
        Rotation2d angle = getAbsoluteHeading();
        return new SwerveModuleState(velocity, angle);
    }

    /**
     * @return Constants about this module, like motor IDs, cancoder angle offset, and translation from center
     */
    public SwerveConstants.Module getModuleConstants(){
        return moduleConstants;
    }

    public void log(){
        SwerveModuleState state = getAbsoluteState();
        int num = moduleConstants.moduleNum;

        SmartDashboard.putNumber("Module "+num+"/Steer Degrees", state.angle.getDegrees());
        SmartDashboard.putNumber("Module "+num+"/Steer Target Degrees", lastDesiredState.angle.getDegrees());
        SmartDashboard.putNumber("Module "+num+"/Steer Setpoint Degrees", Units.radiansToDegrees(steerController.getSetpoint().position));
        SmartDashboard.putNumber("Module "+num+"/Steer Velocity Degrees", Units.radiansToDegrees(getSteerVelocityRadians()));
        SmartDashboard.putNumber("Module "+num+"/Steer Target Velocity Degrees", Units.radiansToDegrees(steerController.getSetpoint().velocity));
        SmartDashboard.putNumber("Module "+num+"/Drive Velocity Feet", Units.metersToFeet(state.speedMetersPerSecond));
        SmartDashboard.putNumber("Module "+num+"/Drive Velocity Target Feet", Units.metersToFeet(lastDesiredState.speedMetersPerSecond));
    }


    // Simulation
    private final TalonFXSimCollection driveMotorSim;
    private final FlywheelSim driveWheelSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            kDriveFF.kv * kWheelCircumference / (2*Math.PI),
            kDriveFF.ka * kWheelCircumference / (2*Math.PI)
        ),
        DCMotor.getFalcon500(1),
        kDriveGearRatio
    );
    private final TalonFXSimCollection steerMotorSim;
    private final FlywheelSim steeringSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(kSteerFF.kv, kSteerFF.ka),
        DCMotor.getFalcon500(1),
        kSteerGearRatio
    );
    private final CANCoderSimCollection steerEncoderSim;

    public void simulationPeriodic(){
        double driveVoltage = driveMotor.getMotorOutputVoltage();
        if(driveVoltage>0) driveVoltage = Math.max(0, driveVoltage-kDriveFF.ks);
        if(driveVoltage<0) driveVoltage = Math.min(0, driveVoltage+kDriveFF.ks);
        driveWheelSim.setInputVoltage(driveVoltage);
        double steerVoltage = steerMotor.getMotorOutputVoltage();
        if(steerVoltage>0) steerVoltage = Math.max(0, steerVoltage-kSteerFF.ks);
        if(steerVoltage<0) steerVoltage = Math.min(0, steerVoltage+kSteerFF.ks);
        steeringSim.setInputVoltage(steerVoltage);
        driveWheelSim.update(0.02);
        steeringSim.update(0.02);

        //SmartDashboard.putNumber("Drive Sim Model Amps", driveWheelSim.getCurrentDrawAmps());
        //SmartDashboard.putNumber("Drive Sim Model Velocity Feet", Units.metersToFeet(driveWheelSim.getAngularVelocityRPM() * kWheelCircumference / 60));
        double driveMotorVelocityNative = TalonUtil.rotationsToVelocity(driveWheelSim.getAngularVelocityRPM()/60, kDriveGearRatio);
        double driveMotorPositionDeltaNative = driveMotorVelocityNative*10*0.02;
        driveMotorSim.setIntegratedSensorVelocity((int)driveMotorVelocityNative);
        driveMotorSim.addIntegratedSensorPosition((int)(driveMotorPositionDeltaNative));
        driveMotorSim.setSupplyCurrent(driveWheelSim.getCurrentDrawAmps());

        //SmartDashboard.putNumber("Steer Sim Model Velocity", steeringSim.getAngularVelocityRPM());
        double steerMotorVelocityNative = TalonUtil.rotationsToVelocity(steeringSim.getAngularVelocityRPM()/60, kSteerGearRatio);
        double steerMotorPositionDeltaNative = steerMotorVelocityNative*10*0.02;
        steerMotorSim.setIntegratedSensorVelocity((int)steerMotorVelocityNative);
        steerMotorSim.addIntegratedSensorPosition((int)(steerMotorPositionDeltaNative));
        steerMotorSim.setSupplyCurrent(steeringSim.getCurrentDrawAmps());
        
        steerEncoderSim.setVelocity((int)(TalonUtil.rotationsToVelocity(steeringSim.getAngularVelocityRPM()/60, 1)*2));
        steerEncoderSim.setRawPosition((int)(getIntegratedHeading().getDegrees()/360.0*4096));

        driveMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        steerMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        steerEncoderSim.setBusVoltage(RobotController.getBatteryVoltage());
    }
}
