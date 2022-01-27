package frc.robot.subsystems;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.SwerveModule;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.SwerveConstants;

public class Drivetrain extends SubsystemBase {

    private final SwerveModule[] swerveMods;
    private final WPI_PigeonIMU gyro;
    private final BasePigeonSimCollection gyroSim; // simulate pigeon

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();

    // path controller and its dimension-specific controllers
    // i.e 1 meter error in the x direction = 1 meter per second x velocity added
    private final PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    private final PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0,
        AutoConstants.kThetaControllerConstraints
    );
    private final HolonomicDriveController pathController = new HolonomicDriveController(xController, yController, thetaController);
    
    public Drivetrain() {
        swerveMods = new SwerveModule[]{
            new SwerveModule(SwerveConstants.Module.FL),
            new SwerveModule(SwerveConstants.Module.FR),
            new SwerveModule(SwerveConstants.Module.BL),
            new SwerveModule(SwerveConstants.Module.BR)
        };

        gyro = new WPI_PigeonIMU(SwerveConstants.kPigeonID);
        gyro.configAllSettings(SwerveConstants.pigeonConfig);
        gyroSim = gyro.getSimCollection();
        zeroGyro();

        kinematics = new SwerveDriveKinematics(
            swerveMods[0].getModuleConstants().centerOffset,
            swerveMods[1].getModuleConstants().centerOffset,
            swerveMods[2].getModuleConstants().centerOffset,
            swerveMods[3].getModuleConstants().centerOffset
        );
        odometry = new SwerveDriveOdometry(kinematics, getGyroYaw());

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        pathController.setEnabled(true); // disable for feedforward-only auto
    }

    @Override
    public void periodic() {
        odometry.update(getGyroYaw(), getModuleStates());
        for(int i=0;i<4;i++){
            swerveMods[i].periodic();
        }
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
        double vx = xPercent * SwerveConstants.kMaxLinearSpeed;
        double vy = yPercent * SwerveConstants.kMaxLinearSpeed;
        double omega = omegaPercent * SwerveConstants.kMaxAngularSpeed;
        ChassisSpeeds targetChassisSpeeds;
        if(fieldRelative){
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getHeading());
        }
        else{
            targetChassisSpeeds = new ChassisSpeeds(vx,vy,omega);
        }
        setChassisSpeeds(targetChassisSpeeds, false);
    }

    public void drive(Trajectory.State targetState, Rotation2d targetRotation){
        // determine ChassisSpeeds from path state and positional feedback control from HolonomicDriveController
        ChassisSpeeds targetChassisSpeeds = pathController.calculate(
            getPose(),
            targetState,
            targetRotation
        );
        // command robot to reach the target ChassisSpeeds
        setChassisSpeeds(targetChassisSpeeds, false);
    }

    /**
     * Command the swerve modules to the desired states.
     * Velocites above maximum speed will be downscaled (preserving ratios between modules)
     * @param steerInPlace If modules should steer to target angle when target velocity is 0
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean steerInPlace){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxLinearSpeed);
        for(int i=0;i<4;i++){
            swerveMods[i].setDesiredState(desiredStates[i], steerInPlace);
        }
    }
    /**
     * Uses kinematics to convert ChassisSpeeds to module states.
     * @see {@link #setModuleStates(SwerveModuleState[], boolean)}
     */
    public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds, boolean steerInPlace){
        setModuleStates(kinematics.toSwerveModuleStates(targetChassisSpeeds), steerInPlace);
        this.targetChassisSpeeds = targetChassisSpeeds;
    }

    public void setBrakeOn(boolean is){
        for(SwerveModule mod : swerveMods){
            mod.setDriveBrake(is);
            mod.setSteerBrake(is);
        }
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }
    public void resetOdometry(Pose2d pose){
        odometry.resetPosition(pose, getGyroYaw());
    }
    public void resetPathController(){
        xController.reset();
        yController.reset();
        thetaController.reset(getHeading().getRadians(), getChassisSpeeds().omegaRadiansPerSecond);
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
    
    /**
     * @return An ordered array filled with module states (rotation, velocity)
     */
    public SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[]{
            swerveMods[0].getState(),
            swerveMods[1].getState(),
            swerveMods[2].getState(),
            swerveMods[3].getState()
        };
    }
    /**
     * @return An ordered array filled with the module field poses 
     */
    public Pose2d[] getModulePoses(){
        Pose2d[] modulePoses = new Pose2d[4];
        for(int i=0;i<4;i++){
            SwerveModule module = swerveMods[i];
            modulePoses[i] = getPose().transformBy(new Transform2d(module.getModuleConstants().centerOffset, module.getIntegratedHeading()));
        }
        return modulePoses;
    }
    public SwerveDriveKinematics getKinematics(){
        return kinematics;
    }
    public ChassisSpeeds getChassisSpeeds(){
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public HolonomicDriveController getPathController(){
        return pathController;
    }

    public void log(){
        Pose2d pose = getPose();
        SmartDashboard.putNumber("Robot Heading", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Robot X", pose.getX());
        SmartDashboard.putNumber("Robot Y", pose.getY());
        ChassisSpeeds chassisSpeeds = getChassisSpeeds();
        SmartDashboard.putNumber("Robot VX", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Robot VY", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Robot Omega Degrees", Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond));
        SmartDashboard.putNumber("Robot Target VX", targetChassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Robot Target VY", targetChassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Robot Target Omega Degrees", Math.toDegrees(targetChassisSpeeds.omegaRadiansPerSecond));
        
        for(int i=0;i<4;i++){
            SwerveModule module = swerveMods[i];
            module.log();
        }
    }

    @Override
    public void simulationPeriodic(){
        for(int i=0;i<4;i++){
            SwerveModule module = swerveMods[i];
            module.simulationPeriodic();
        }

        double chassisOmega = getChassisSpeeds().omegaRadiansPerSecond;
        chassisOmega = Math.toDegrees(chassisOmega);
        gyroSim.addHeading(chassisOmega*0.02);
    }
}
