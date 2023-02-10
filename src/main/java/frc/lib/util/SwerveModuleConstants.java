package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int absAngleEncoderID;
    public final Rotation2d angleOffset2d;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param absAngleEncoderID
     * @param angleOffset2d
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int absAngleEncoderID, Rotation2d angleOffset2d) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.absAngleEncoderID = absAngleEncoderID;
        this.angleOffset2d = angleOffset2d;
    }
}
