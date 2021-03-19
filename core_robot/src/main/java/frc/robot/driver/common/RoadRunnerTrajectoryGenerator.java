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

        Path forward5ft = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToLinearHeading(new Vector2d(60, 0.0))
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

// ----------------------------------------- Galactic Search Paths ------------------------------

        double scaleConstant = 0.5;

        Path redPathA = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToLinearHeading(new Vector2d(45.0, 0.0)) // C3
            .splineToConstantHeading(new Pose2d(105.0, -30.0, 0.0), 1.0) // D5     | what's end tangent?
            .splineToConstantHeading(new Pose2d(135.0, 60.0, 0.0), 1.0) // A6
            .lineToLinearHeading(new Vector2d(295.0, 60.0, 0.0)) // end
            .build();
        pathManager.addPath( 
            "redPathA",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(redPathA, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));

        Path bluePathA = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToLinearHeading(new Vector2d(135.0, 0.0)) // E6
            .splineToConstantHeading(new Pose2d(165.0, 90.0, 0.0), 1.0) // B7
            .splineToConstantHeading(new Pose2d(225.0, 60.0, 0.0), 1.0) // C9
            .lineToLinearHeading(new Vector2d(295.0, 60.0)) // end
            .build();
        pathManager.addPath( 
            "bluePathA",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(bluePathA, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));

            
        Path redPathB = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToLinearHeading(new Vector2d(45.0, 0.0)) // B3
            .splineToConstantHeading(new Pose2d(105.0, -60.0, 0.0), 1.0) // D5
            .splineToConstantHeading(new Pose2d(165.0, 0.0, 0.0), 1.0) // B7
            .lineToLinearHeading(new Vector2d(295.0, 0.0)) // end
            .build();
        pathManager.addPath( 
            "redPathB",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(redPathB, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));

        Path bluePathB = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToLinearHeading(new Vector2d(135.0, 0.0)) // D6
            .splineToConstantHeading(new Pose2d(195.0, 60.0, 0.0), 1.0) // B8
            .splineToConstantHeading(new Pose2d(225.0, 0.0, 0.0), 1.0) // D10
            .lineToLinearHeading(new Vector2d(295.0, 0.0)) // end
            .build();
        pathManager.addPath( 
            "bluePathB",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(bluePathB, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));


        /*
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
    
        /**/
        // ----------------------- Slalom paths ------------------ 
        // robot dimensions: 32in x 28in?
        // forward left is positive
        // starting point is with front left corner at D2
        Path slalom = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .splineTo(76.0, 30.0)
            .splineTo(196.0, 30.0)
            .splineTo(256.0, 0.0)
            .splineTo(288.0, 15)
            .splineTo(256.0, 30)
            .splineTo(196.0, 0)
            .splineTo(76.0, 0)
            .splineTo(-10, 0)
            .build();
        pathManager.addPath( 
            "slalom",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(slalom, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));
    
        // ----------------------- barrel race paths ------------------ 
    
        Path barrelRace = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToLinearHeading(new Vector2d(105.0, 0.0))
            .splineTo(122, -15)
            .splineTo(105, -30)
            .splineTo(98, -15) // behind D5
            .splineTo(222, 45)
            .splineTo(205, 60)
            .splineTo(187, 45)
            .splineTo(254, -30)
            .splineTo(272, 0)
            .splineTo(-10, 28)
            .build();
        pathManger.addPath(
            "barrelRace",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(barrelRace, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));
    
    // ----------------------- bounce paths ------------------ 
        Path bounceStart = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .splineTo(48, 46) // first target (A3)
            .lineToConstantHeading(new Vector2d(74, -43))
            .splineTo(104, -63)
            .splineTo(136, 46) // second target (A6)
            .lineToConstantHeading(146, -73)
            .splineTo(181, -60)
            .splineTo(220, 16)
            .lineToConstantHeading(226, 46) // third target (A9)
            .splineTo(278, 0)
            .build();
        pathManger.addPath(
            "bounceStart",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(bounceStart, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));
        )
    
    
    }

}