package frc.robot.VortexSwerveCode.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VortexSwerveCode.Constants;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry _swerveOdometry;
    public SwerveModule[] _swerveMods;
    public AHRS _gyro;

    public Swerve() {
        _gyro = new AHRS();
        zeroGyro();

        _swerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        _swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : _swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod._moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for (SwerveModule mod : _swerveMods) {
            mod.setDesiredState(desiredStates[mod._moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return _swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        _swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : _swerveMods) {
            states[mod._moduleNumber] = mod.getCurrentState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : _swerveMods) {
            positions[mod._moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        _gyro.reset();
        if (_gyro.isCalibrating()) {
            System.out.println("Calibrating...");
        }
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - _gyro.getYaw()) : Rotation2d.fromDegrees(_gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : _swerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        _swerveOdometry.update(getYaw(), getModulePositions());

        for (SwerveModule mod : _swerveMods) {
            SmartDashboard.putNumber("Mod " + mod._moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod._moduleNumber + " Cancoder With Offset", mod.getCanCoderWithOffset().getDegrees());
            SmartDashboard.putNumber("Mod " + mod._moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod._moduleNumber + " Actual Speed", mod.getCurrentState().speedMetersPerSecond);
            SmartDashboard.putNumber("Mod " + mod._moduleNumber + " Desired Speed", mod.getDesiredState().speedMetersPerSecond);
            SmartDashboard.putNumber("Gyroscope Angle", _gyro.getAngle());
            SmartDashboard.putNumber("Distance: ", this.getPose().getX());
        }
    }
}
