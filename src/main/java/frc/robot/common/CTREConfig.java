package frc.robot.common;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

public class CTREConfig {
    
    public TalonFXConfiguration swerveDriveConfig;
    public TalonFXConfiguration swerveSteerConfig;
    public CANCoderConfiguration swerveCancoderConfig;

    public CTREConfig(){
        swerveDriveConfig = new TalonFXConfiguration();
        swerveSteerConfig = new TalonFXConfiguration();
        swerveCancoderConfig = new CANCoderConfiguration();
    }
}
