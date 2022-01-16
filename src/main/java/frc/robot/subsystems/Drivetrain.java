package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.CTREConfigs;
import frc.robot.common.Constants;
import frc.robot.common.SwerveModule;

public class Drivetrain extends SubsystemBase {

    private SwerveModule[] swerveMods;
    private WPI_PigeonIMU gyro;

    private SwerveDriveOdometry odometry;

     // path controller PID controllers
     // i.e 1 meter error in the x direction = 1 meter per second x velocity added
    private PIDController xController = new PIDController(Constants.Auto.kPXController, 0, 0);
    private PIDController yController = new PIDController(Constants.Auto.kPYController, 0, 0);
    private ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.Auto.kPThetaController, 0, 0,
        Constants.Auto.kThetaControllerConstraints
    );
    private HolonomicDriveController pathController; // Auto path-following controller
    
    public Drivetrain() {
        CTREConfigs ctreConfigs = new CTREConfigs();
        swerveMods = new SwerveModule[]{
            new SwerveModule(Constants.Swerve.Module.FL, ctreConfigs),
            new SwerveModule(Constants.Swerve.Module.FR, ctreConfigs),
            new SwerveModule(Constants.Swerve.Module.BL, ctreConfigs),
            new SwerveModule(Constants.Swerve.Module.BR, ctreConfigs)
        };

        gyro = new WPI_PigeonIMU(Constants.Swerve.kPigeonID);
        gyro.configFactoryDefault(30);
        zeroGyro();

        odometry = new SwerveDriveOdometry(Constants.Swerve.kinematics, getGyroYaw());

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        pathController = new HolonomicDriveController(xController, yController, thetaController);
    }

    @Override
    public void periodic() {
        odometry.update(getGyroYaw(), getModuleStates());
    }

    /**
     * Basic teleop drive control; percentages representing vx, vy, and omega
     * are converted to chassis speeds for the robot to follow
     * @param xPercent vx (forward)
     * @param yPercent vy (strafe)
     * @param omegaPercent omega (rotation CCW+)
     * @param fieldRelative If is field-relative control
     */
    public void drive(double xPercent, double yPercent, double omegaPercent, boolean fieldRelative){
        double vx = xPercent * Constants.Swerve.kMaxLinearSpeed;
        double vy = yPercent * Constants.Swerve.kMaxLinearSpeed;
        double omega = omegaPercent * Constants.Swerve.kMaxAngularSpeed;
        ChassisSpeeds targetChassisSpeeds;
        if(fieldRelative){
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getHeading());
        }
        else{
            targetChassisSpeeds = new ChassisSpeeds(vx,vy,omega);
        }
        setModuleStates(Constants.Swerve.kinematics.toSwerveModuleStates(targetChassisSpeeds), false);
    }

    /**
     * Command the swerve modules to the desired states.
     * Velocites above maximum speed will be downscaled (preserving ratios between modules)
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean spinInPlace){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.kMaxLinearSpeed);
        for(int i=0;i<4;i++){
            swerveMods[i].setDesiredState(desiredStates[i], spinInPlace);
        }
    }

    public void setBrakeOn(boolean is){
        for(SwerveModule mod : swerveMods){
            mod.setDriveBrake(is);
        }
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }
    public void resetOdometry(Pose2d pose){
        odometry.resetPosition(pose, getGyroYaw());
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }
    public Rotation2d getHeading(){
        return odometry.getPoseMeters().getRotation();
    }
    public Rotation2d getGyroYaw(){
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return Rotation2d.fromDegrees(ypr[0]);
    }

    public SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[]{
            swerveMods[0].getState(),
            swerveMods[1].getState(),
            swerveMods[2].getState(),
            swerveMods[3].getState()
        };
    }

    public HolonomicDriveController getPathController(){
        return pathController;
    }

    public void log(){
        SmartDashboard.putNumber("Gyro Degrees", getGyroYaw().getDegrees());
        ChassisSpeeds chassisSpeeds = Constants.Swerve.kinematics.toChassisSpeeds(getModuleStates());
        SmartDashboard.putNumber("Linear Velocity Feet", Units.metersToFeet(Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)));
        SmartDashboard.putNumber("Angular Velocity Degrees", Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond));
        for(int i=0;i<4;i++){
            swerveMods[i].log(i);
        }
    }
}
