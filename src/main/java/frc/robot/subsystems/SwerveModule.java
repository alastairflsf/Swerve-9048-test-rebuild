package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Constants.DriveConstants.*;

public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    private final CANcoder absoluteEncoder;
    private final double absoluteEncoderOffsetRad;

    private final SimpleMotorFeedforward driveFeedforward =
            new SimpleMotorFeedforward(0.0, kDriveFF);

    public SwerveModule(int driveMotorId, int turnMotorId, int canCoderId,
                         double encoderOffsetRad, boolean driveInverted) {

        this.absoluteEncoderOffsetRad = encoderOffsetRad;
        this.absoluteEncoder = new CANcoder(canCoderId);

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new SparkMax(turnMotorId, MotorType.kBrushless);

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.inverted(driveInverted);
        driveConfig.smartCurrentLimit(50);
        driveConfig.encoder
                .positionConversionFactor(kDrivePositionConversionFactor)
                .velocityConversionFactor(kDriveVelocityConversionFactor);
        driveConfig.closedLoop
                .p(kDriveP)
                .i(kDriveI)
                .d(kDriveD);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getClosedLoopController();

        SparkMaxConfig turnConfig = new SparkMaxConfig();
        turnConfig.inverted(kTurnMotorInvertedMK4i);
        turnConfig.smartCurrentLimit(20);
        turnConfig.encoder.positionConversionFactor(kTurnPositionConversionFactor);
        turnConfig.closedLoop
                .p(kTurnP)
                .i(kTurnI)
                .d(kTurnD)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI);
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turnEncoder = turnMotor.getEncoder();
        turnController = turnMotor.getClosedLoopController();

        resetToAbsolute();
    }

    public void resetToAbsolute() {
        double absoluteAngleRad =
                absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI - absoluteEncoderOffsetRad;
        turnEncoder.setPosition(absoluteAngleRad);
    }

    public double getDrivePositionMeters() {
        return driveEncoder.getPosition();
    }

    public double getDriveVelocityMetersPerSecond() {
        return driveEncoder.getVelocity();
    }

    public Rotation2d getTurnAngle() {
        return new Rotation2d(turnEncoder.getPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getTurnAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getTurnAngle());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState.optimize(getTurnAngle());

        turnController.setReference(desiredState.angle.getRadians(),
                SparkMax.ControlType.kPosition);

        double ffVolts = driveFeedforward.calculate(desiredState.speedMetersPerSecond);
        driveController.setReference(desiredState.speedMetersPerSecond,
                SparkMax.ControlType.kVelocity, 0, ffVolts);
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
