package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.DriveConstants.*;

public class RobotContainer {

    private final SwerveSubsystem swerve = new SwerveSubsystem();

    private final CommandPS5Controller driverController =
            new CommandPS5Controller(kDriverControllerPort);

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new RunCommand(() -> {

            double xSpeed = -applyDeadband(driverController.getLeftY()) * kMaxSpeedMetersPerSecond;
            double ySpeed = -applyDeadband(driverController.getLeftX()) * kMaxSpeedMetersPerSecond;
            double rotSpeed = -applyDeadband(driverController.getRightX()) * kMaxAngularSpeedRadPerSec;

            swerve.drive(xSpeed, ySpeed, rotSpeed, true);

        }, swerve));
    }

    private void configureButtonBindings() {
        driverController.options().onTrue(
                new RunCommand(swerve::zeroGyro, swerve).withTimeout(0.02)
        );
    }

    private double applyDeadband(double value) {
        return MathUtil.applyDeadband(value, kJoystickDeadband);
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
