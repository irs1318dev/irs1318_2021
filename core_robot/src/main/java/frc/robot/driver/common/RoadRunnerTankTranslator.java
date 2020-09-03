package frc.robot.driver.common;

import java.util.ArrayList;
import java.util.List;

import java.io.File;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.TankKinematics;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints;

import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.common.*;
import frc.robot.driver.PathManager;

public class RoadRunnerTankTranslator
{
    public static final String filePath = System.getProperty("user.home") + File.separator + "path.csv";

    public static void main(String[] args)
    {
        // Initialize path and interpolator (check to see if interpolator is needed)
        Path path =  new PathBuilder(new Pose2d(132, 0, 0)) 
            .splineTo(new Pose2d(0, 200, 90)) // tune y value and angle for shooting position (90 might need to be 270) 
            .build();

        boolean isBackwards = true;

        PathManager.writePathToFile(
            RoadRunnerTankTranslator.filePath,
            RoadRunnerTankTranslator.convert(path, isBackwards));
    }

    public static List<PathStep> convert(Path path, boolean isBackwards)
    {
        return null;
    }

    public static PathStep buildPathStep(double time, Trajectory traj, double leftWheelPos, double rightWheelPos, boolean isBackwards)
    {
        return null;
    }
}