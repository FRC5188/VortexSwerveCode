package frc.robot.VortexSwerveCode.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int _driveMotorID;
    public final int _angleMotorID;
    public final int _cancoderID;
    public final Rotation2d _angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset) {
        this._driveMotorID = driveMotorID;
        this._angleMotorID = angleMotorID;
        this._cancoderID = canCoderID;
        this._angleOffset = angleOffset;
    }
}
