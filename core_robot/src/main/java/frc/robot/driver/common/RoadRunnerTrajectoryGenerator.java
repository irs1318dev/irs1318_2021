package frc.robot.driver.common;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;

import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.path.*;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.constraints.*;

import de.siegmar.fastcsv.writer.CsvWriter;
import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.common.*;
import frc.robot.common.robotprovider.ITrajectory;
import frc.robot.common.robotprovider.TrajectoryState;
import frc.robot.driver.PathManager;

public class RoadRunnerTrajectoryGenerator
{
    private static final TrajectoryVelocityConstraint velocityConstraint =
        new MinVelocityConstraint(
            Arrays.asList(
                new SwerveVelocityConstraint(
                    TuningConstants.DRIVETRAIN_MAX_MODULE_PATH_VELOCITY,
                    HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE,
                    HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE),
                new AngularVelocityConstraint(TuningConstants.DRIVETRAIN_MAX_PATH_TURN_VELOCITY * Helpers.DEGREES_TO_RADIANS),
                new TranslationalVelocityConstraint(TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY)));

    private static final TrajectoryAccelerationConstraint accelerationConstraint =
            new ProfileAccelerationConstraint(TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION);

    public static void main(String[] args)
    {
        Path turnArcLeft = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .splineToSplineHeading(new Pose2d(72.0, 40.0, 90.0 * Helpers.DEGREES_TO_RADIANS), 90.0 * Helpers.DEGREES_TO_RADIANS)
            .splineToSplineHeading(new Pose2d(100.0, 90.0, 0.0 * Helpers.DEGREES_TO_RADIANS), 0.0 * Helpers.DEGREES_TO_RADIANS)
            .build();

        ITrajectory trajectory = new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(turnArcLeft, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint));

        try (CsvWriter csvWriter = CsvWriter.builder().build(java.nio.file.Path.of("test.csv"), StandardCharsets.UTF_8))
        {
            csvWriter.writeRow("x", "y", "theta", "vx", "vy", "omega");

            for (double t = 0.0; t < trajectory.getDuration() + 0.01; t += 0.02)
            {
                TrajectoryState state = trajectory.get(t);
                csvWriter.writeRow(
                    Double.toString(state.xPosition),
                    Double.toString(state.yPosition),
                    Double.toString(state.angle),
                    Double.toString(state.xVelocity),
                    Double.toString(state.yVelocity),
                    Double.toString(state.angleVelocity));
            }

            csvWriter.close();
        }
        catch (IOException e)
        {
        }
    }

    public static void generateTrajectories(PathManager pathManager)
    {
        Path turnArcLeft = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .splineToLinearHeading(new Pose2d(72.0, 40.0, 90.0 * Helpers.DEGREES_TO_RADIANS), 1.0)
            .build();
        pathManager.addPath(
            "turnArcLeft",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(turnArcLeft, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));

        // -------------------- path A paths ----------------   

        Path forward5ft = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(60.0, 0.0))
            .build();
        pathManager.addPath(
            "forward5ft",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(forward5ft, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));

        Path left5ft = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(0.0, 60.0))
            .build();
        pathManager.addPath(
            "left5ft",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(left5ft, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));

        Path back5ft = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(-60.0, 0.0))
            .build();
        pathManager.addPath(
            "back5ft",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(back5ft, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));

        Path right5ft = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(0.0, -60.0))
            .build();
        pathManager.addPath(
            "right5ft",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(right5ft, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));

        // D5 TO A6, E6 TO B7
        Path slideToTheLeft = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(-30.0, 90.0))
            .build();
        pathManager.addPath( 
            "slideToTheLeft",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(slideToTheLeft, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));

        // C3 TO D5, B7 TO C9 
        Path slideToTheRight = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(60.0, -30.0))
            .build();
        pathManager.addPath( 
            "slideToTheRight",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(slideToTheRight, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));
        
        // A6 TO A11, E1 TO E6
        Path crissCross = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(150.0, 0.0))
            .build();
        pathManager.addPath( 
            "crissCross",                // EVERYBODY CLAP YOUR HANDS  
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(crissCross, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));


        // ----------------------- Path B paths ------------------  // SLIDE AT YOUR OWN RISK

        // B1 TO B3: forward5feet
        Path slideToTheRightB = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(60.0, -60.0))
            .build();
        pathManager.addPath( //B3 TO D5, B8 TO D10
            "slideToTheRightB",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(slideToTheRightB, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));

        Path slideToTheLeftB = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(60.0, 60.0))
            .build();
        pathManager.addPath( // D5 TO B7
            "slideToTheLeftB",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(slideToTheLeftB, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));

        Path chaChaNowYall = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(30.0, 60.0))
            .build();
        pathManager.addPath( //D10 to D11, 
            "chaChaNowYall", // go forward a lil bit - it'll be a smol amount, almost as smol as VaruANsShiHIKA 
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(chaChaNowYall, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));

        Path chaChaRealSmooth = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(120.0, 60.0))
            .build();
        pathManager.addPath( //B7 to B11
            "chaChaRealSmooth", // goes forward 10 ft
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(chaChaRealSmooth, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));
    }
}