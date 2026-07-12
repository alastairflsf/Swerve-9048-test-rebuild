package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kJoystickDeadband = 0.06;
    }

    public static final class DriveConstants {

        // TODO: confirmar no REV Hardware Client / Phoenix Tuner
        public static final int kFrontLeftDriveMotorId = 1;
        public static final int kFrontLeftTurnMotorId = 2;
        public static final int kFrontLeftCANcoderId = 9;

        public static final int kFrontRightDriveMotorId = 3;
        public static final int kFrontRightTurnMotorId = 4;
        public static final int kFrontRightCANcoderId = 10;

        public static final int kBackLeftDriveMotorId = 5;
        public static final int kBackLeftTurnMotorId = 6;
        public static final int kBackLeftCANcoderId = 11;

        public static final int kBackRightDriveMotorId = 7;
        public static final int kBackRightTurnMotorId = 8;
        public static final int kBackRightCANcoderId = 12;

        public static final int kPigeonId = 13;

        // MK4i: motor de rotação vem invertido de fábrica em relação ao MK4
        public static final boolean kTurnMotorInvertedMK4i = true;

        // TODO: calibrar offsets (girar rodas pra frente, ler CANcoder no Tuner X)
        public static final double kFrontLeftEncoderOffsetRad = 0.0;
        public static final double kFrontRightEncoderOffsetRad = 0.0;
        public static final double kBackLeftEncoderOffsetRad = 0.0;
        public static final double kBackRightEncoderOffsetRad = 0.0;

        public static final boolean kFrontLeftDriveInverted = false;
        public static final boolean kFrontRightDriveInverted = false;
        public static final boolean kBackLeftDriveInverted = false;
        public static final boolean kBackRightDriveInverted = false;

        // TODO: medir no chassi
        public static final double kTrackWidthMeters = Units.inchesToMeters(24);
        public static final double kWheelBaseMeters = Units.inchesToMeters(24);

        // TODO: medir roda com o treado (padrão MK4i ~4in)
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        // TODO: confirmar redução do módulo (L1 = 8.14 | L2 = 6.75 | L3 = 6.12) — padrão L2
        public static final double kDriveGearRatio = 6.75;

        // Fixo em todo MK4i
        public static final double kTurnGearRatio = 150.0 / 7.0;

        public static final double kDrivePositionConversionFactor =
                kWheelCircumferenceMeters / kDriveGearRatio;
        public static final double kDriveVelocityConversionFactor =
                kDrivePositionConversionFactor / 60.0;

        public static final double kTurnPositionConversionFactor =
                2 * Math.PI / kTurnGearRatio;

        // TODO: tunar no robô
        public static final double kTurnP = 0.5;
        public static final double kTurnI = 0.0;
        public static final double kTurnD = 0.0;

        public static final double kDriveP = 0.05;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveFF = 1.0 / 5676.0;

        public static final double kMaxSpeedMetersPerSecond = 4.0;
        public static final double kMaxAngularSpeedRadPerSec = 2 * Math.PI;

        public static final Translation2d kFrontLeftLocation =
                new Translation2d(kWheelBaseMeters / 2, kTrackWidthMeters / 2);
        public static final Translation2d kFrontRightLocation =
                new Translation2d(kWheelBaseMeters / 2, -kTrackWidthMeters / 2);
        public static final Translation2d kBackLeftLocation =
                new Translation2d(-kWheelBaseMeters / 2, kTrackWidthMeters / 2);
        public static final Translation2d kBackRightLocation =
                new Translation2d(-kWheelBaseMeters / 2, -kTrackWidthMeters / 2);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation
        );
    }
}
