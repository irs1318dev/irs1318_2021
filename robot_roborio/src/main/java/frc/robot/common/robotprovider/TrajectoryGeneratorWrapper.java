package frc.robot.common.robotprovider;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.common.Helpers;

public class TrajectoryGeneratorWrapper implements ITrajectoryGenerator
{
    private final SwerveDriveKinematics swerveDriveKinematics;
    private final SwerveDriveKinematicsConstraint swerveConstraint;
    private final TrajectoryConfig swerveConfig;

    public TrajectoryGeneratorWrapper()
    {
        double a = (HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 12.0) / 2.0;
        double b = (HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 12.0) / 2.0;

        this.swerveDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(Units.feetToMeters(-a), Units.feetToMeters(-b)),
                new Translation2d(Units.feetToMeters(a), Units.feetToMeters(-b)),
                new Translation2d(Units.feetToMeters(a), Units.feetToMeters(b)),
                new Translation2d(Units.feetToMeters(-a), Units.feetToMeters(b)));

        this.swerveConstraint =
            new SwerveDriveKinematicsConstraint(
                this.swerveDriveKinematics,
                Units.feetToMeters(TuningConstants.DRIVETRAIN_MAX_MODULE_PATH_VELOCITY / 12.0));

        this.swerveConfig =
            new TrajectoryConfig(
                Units.feetToMeters(TuningConstants.DRIVETRAIN_MAX_PATH_VELOCITY / 12.0),
                Units.feetToMeters(TuningConstants.DRIVETRAIN_MAX_PATH_ACCELERATION / 12.0));
        this.swerveConfig.addConstraint(this.swerveConstraint);
    }

    public ITrajectory generateTrajectory(Pose2d start, Pose2d end, Point2d[] translations)
    {
        edu.wpi.first.wpilibj.geometry.Pose2d sideStart = new edu.wpi.first.wpilibj.geometry.Pose2d(start.x * Helpers.METERS_PER_INCH, start.y * Helpers.METERS_PER_INCH, Rotation2d.fromDegrees(start.angle));
        edu.wpi.first.wpilibj.geometry.Pose2d crossScale = new edu.wpi.first.wpilibj.geometry.Pose2d(end.x * Helpers.METERS_PER_INCH, end.y * Helpers.METERS_PER_INCH, Rotation2d.fromDegrees(end.angle));

        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>(translations.length);
        for (int i = 0; i < translations.length; i++)
        {
            interiorWaypoints.add(new Translation2d(translations[i].x * Helpers.METERS_PER_INCH, translations[i].y * Helpers.METERS_PER_INCH));
        }

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            sideStart,
            interiorWaypoints,
            crossScale,
            this.swerveConfig);

        return new TrajectoryWrapper(trajectory);
    }
}
