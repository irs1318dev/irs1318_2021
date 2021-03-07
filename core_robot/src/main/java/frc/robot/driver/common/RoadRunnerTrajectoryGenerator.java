package frc.robot.driver.common;

import java.util.Arrays;

import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.path.*;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.constraints.*;

import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.common.*;
import frc.robot.driver.PathManager;

public class RoadRunnerTrajectoryGenerator
{
    public static void main(String[] args)
    {
    }

    public static void generateTrajectories(PathManager pathManager)
    {
        TrajectoryVelocityConstraint velocityConstraint =
            new MinVelocityConstraint(
                Arrays.asList(
                    new SwerveVelocityConstraint(
                        TuningConstants.DRIVETRAIN_MAX_MODULE_PATH_VELOCITY,
                        HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE,
                        HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE),
                    new AngularVelocityConstraint(TuningConstants.DRIVETRAIN_MAX_PATH_TURN_VELOCITY * Helpers.DEGREES_TO_RADIANS),
                    new TranslationalVelocityConstraint(TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY)));

        TrajectoryAccelerationConstraint accelerationConstraint =
            new ProfileAccelerationConstraint(TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION);

        Path turnArcLeft = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToLinearHeading(new Pose2d(100.0, 100.0, 90.0 * Helpers.DEGREES_TO_RADIANS))
            .build();
        pathManager.addPath(
            "turnArcLeft",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(turnArcLeft, velocityConstraint, accelerationConstraint)));

        // -------------------- path A paths ----------------   

        Path forward5ft = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(60.0, 60.0))
            .build();
        pathManager.addPath(
            "forward5ft",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(forward5ft, velocityConstraint, accelerationConstraint)));

        // D5 TO A6, E6 TO B7
        Path slideToTheLeft = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(30.0, 90.0))
            .build();
        pathManager.addPath( 
            "slideToTheLeft",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(slideToTheLeft, velocityConstraint, accelerationConstraint)));

        // C3 TO D5, B7 TO C9 
        Path slideToTheRight = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(60.0, -30.0))
            .build();
        pathManager.addPath( 
            "slideToTheRight",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(slideToTheRight, velocityConstraint, accelerationConstraint)));
        
        // A6 TO A11, E1 TO E6
        Path crissCross = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(150.0, 0.0))
            .build();
        pathManager.addPath( 
            "crissCross",                // EVERYBODY CLAP YOUR HANDS  
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(crissCross, velocityConstraint, accelerationConstraint)));


        // ----------------------- Path B paths ------------------  // SLIDE AT YOUR OWN RISK

        // B1 TO B3: forward5feet
        Path slideToTheRightB = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(60.0, -60.0))
            .build();
        pathManager.addPath( //B3 TO D5, B8 TO D10
            "slideToTheRightB",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(slideToTheRightB, velocityConstraint, accelerationConstraint)));

        Path slideToTheLeftB = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(60.0, 60.0))
            .build();
        pathManager.addPath( // D5 TO B7
            "slideToTheLeftB",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(slideToTheLeftB, velocityConstraint, accelerationConstraint)));

        Path chaChaNowYall = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(30.0, 60.0))
            .build();
        pathManager.addPath( //D10 to D11, 
            "chaChaNowYall", // go forward a lil bit - it'll be a smol amount, almost as smol as VaruANsShiHIKA 
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(chaChaNowYall, velocityConstraint, accelerationConstraint)));

        Path chaChaRealSmooth = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .lineToConstantHeading(new Vector2d(120.0, 60.0))
            .build();
        pathManager.addPath( //B7 to B11
            "chaChaRealSmooth", // goes forward 10 ft
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(chaChaRealSmooth, velocityConstraint, accelerationConstraint)));
    }
}