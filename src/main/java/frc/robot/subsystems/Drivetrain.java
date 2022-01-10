package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    // Constants
    public static final double kTrackWidth = Units.inchesToMeters(20);
    public static final double kTrackLength = Units.inchesToMeters(20);
    
    public static final double kMaxLinearSpeed = 4;
    public static final double kMaxAngularSpeed = 2*Math.PI;
    
    
    
    public Drivetrain() {}
    
    @Override
    public void periodic() {
    }
}
