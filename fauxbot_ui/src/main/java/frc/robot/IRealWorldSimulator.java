package frc.robot;

import java.util.Set;

import frc.robot.common.robotprovider.*;
import javafx.scene.canvas.Canvas;

public interface IRealWorldSimulator
{
    public FauxbotSensorConnection[] getSensors();
    public FauxbotActuatorConnection[] getActuators();
    public String getSensorName(FauxbotSensorConnection connection);
    public double getSensorMin(FauxbotSensorConnection connection);
    public double getSensorMax(FauxbotSensorConnection connection);
    public String getActuatorName(FauxbotActuatorConnection connection);
    public double getMotorMin(FauxbotActuatorConnection connection);
    public double getMotorMax(FauxbotActuatorConnection connection);
    public void update();
    public void draw(Canvas canvas);
}
