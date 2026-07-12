package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveConstants.*;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
            kFrontLeftDriveMotorId, kFrontLeftTurnMotorId, kFrontLeftCANcoderId,
            kFrontLeftEncoderOffsetRad, kFrontLeftDriveInverted);

    private final SwerveModule frontRight = new SwerveModule(
            kFrontRightDriveMotorId, kFrontRightTurnMotorId, kFrontRightCANcoderId,
            kFrontRightEncoderOffsetRad, kFrontRightDriveInverted);

    private final SwerveModule backLeft = new SwerveModule(
            kBackLeftDriveMotorId, kBackLeftTurnMotorId, kBackLeftCANcoderId,
            kBackLeftEncoderOffsetRad, kBackLeftDriveInverted);

    private final SwerveModule backRight = new SwerveModule(
            kBackRightDriveMotorId, kBackRightTurnMotorId, kBackRightCANcoderId,
            kBackRightEncoderOffsetRad, kBackRightDriveInverted);

    private final Pigeon2 gyro = new Pigeon2(kPigeonId);

    private final SwerveDriveOdometry odometry;

    public SwerveSubsystem() {
        gyro.reset();

        odometry = new SwerveDriveOdometry(
                kDriveKinematics,
                getGyroYaw(),
                getModulePositions()
        );
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void zeroGyro() {
        gyro.reset();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getGyroYaw())
                : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);

        SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    @Override
    public void periodic() {
        odometry.update(getGyroYaw(), getModulePositions());

        SmartDashboard.putNumber("Gyro Yaw (deg)", getGyroYaw().getDegrees());
        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
    }
}
