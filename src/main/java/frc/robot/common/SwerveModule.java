package frc.robot.common;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public class SwerveModule {

    private final int driveMotorID;
    private final int steerMotorID;
    private final int cancoderID;
    private final double angleOffset;

    // Hardware
    private WPI_TalonFX driveMotor;
    private WPI_TalonFX steerMotor;
    private WPI_CANCoder steerEncoder;   

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
