package frc.robot.common.robotprovider;

import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;;

public class VideoStreamWrapper implements IVideoStream
{
    public final CvSource wrappedObject;

    public VideoStreamWrapper(String name, int width, int height)
    {
        this.wrappedObject = CameraServer.putVideo(name, width, height);
    }

    public void putFrame(IMat image)
    {
        this.wrappedObject.putFrame(OpenCVProvider.unwrap(image));
    }
}