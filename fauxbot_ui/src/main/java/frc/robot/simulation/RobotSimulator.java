package frc.robot.simulation;

import java.util.Calendar;
import java.util.HashMap;
import java.util.Map;

import frc.robot.IRealWorldSimulator;
import frc.robot.common.robotprovider.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import javafx.scene.canvas.Canvas;

@Singleton
public class RobotSimulator implements IRealWorldSimulator
{
    private static final FauxbotActuatorConnection AngleMotor1Connection = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, 1);
    private static final FauxbotActuatorConnection DriveMotor1Connection = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, 2);
    private static final FauxbotSensorConnection AngleEncoder1Connection = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.CAN, 1);
    private static final FauxbotSensorConnection DriveEncoder1Connection = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.CAN, 2);
    private static final FauxbotActuatorConnection AngleMotor2Connection = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, 3);
    private static final FauxbotActuatorConnection DriveMotor2Connection = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, 4);
    private static final FauxbotSensorConnection AngleEncoder2Connection = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.CAN, 3);
    private static final FauxbotSensorConnection DriveEncoder2Connection = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.CAN, 4);
    private static final FauxbotActuatorConnection AngleMotor3Connection = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, 5);
    private static final FauxbotActuatorConnection DriveMotor3Connection = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, 6);
    private static final FauxbotSensorConnection AngleEncoder3Connection = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.CAN, 5);
    private static final FauxbotSensorConnection DriveEncoder3Connection = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.CAN, 6);
    private static final FauxbotActuatorConnection AngleMotor4Connection = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, 7);
    private static final FauxbotActuatorConnection DriveMotor4Connection = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, 8);
    private static final FauxbotSensorConnection AngleEncoder4Connection = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.CAN, 7);
    private static final FauxbotSensorConnection DriveEncoder4Connection = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.CAN, 8);

    private static final FauxbotSensorConnection AbsoluteEncoder1Connection = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.AnalogInput, 0);
    private static final FauxbotSensorConnection AbsoluteEncoder2Connection = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.AnalogInput, 1);
    private static final FauxbotSensorConnection AbsoluteEncoder3Connection = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.AnalogInput, 2);
    private static final FauxbotSensorConnection AbsoluteEncoder4Connection = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.AnalogInput, 3);

    private static final FauxbotSensorConnection NavXConnection = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.NavX, 0);

    private static final FauxbotActuatorConnection[] DriveMotors =
        new FauxbotActuatorConnection[]
        {
            RobotSimulator.DriveMotor1Connection,
            RobotSimulator.DriveMotor2Connection,
            RobotSimulator.DriveMotor3Connection,
            RobotSimulator.DriveMotor4Connection
        };

    private static final FauxbotActuatorConnection[] AngleMotors =
        new FauxbotActuatorConnection[]
        {
            RobotSimulator.AngleMotor1Connection,
            RobotSimulator.AngleMotor2Connection,
            RobotSimulator.AngleMotor3Connection,
            RobotSimulator.AngleMotor4Connection
        };

    private static final FauxbotSensorConnection[] DriveEncoders =
        new FauxbotSensorConnection[]
        {
            RobotSimulator.DriveEncoder1Connection,
            RobotSimulator.DriveEncoder2Connection,
            RobotSimulator.DriveEncoder3Connection,
            RobotSimulator.DriveEncoder4Connection
        };

    private static final FauxbotSensorConnection[] AngleEncoders =
        new FauxbotSensorConnection[]
        {
            RobotSimulator.AngleEncoder1Connection,
            RobotSimulator.AngleEncoder2Connection,
            RobotSimulator.AngleEncoder3Connection,
            RobotSimulator.AngleEncoder4Connection
        };

    private final FauxbotSensorConnection[] sensors =
        new FauxbotSensorConnection[]
        {
            RobotSimulator.AngleEncoder1Connection,
            RobotSimulator.AngleEncoder2Connection,
            RobotSimulator.AngleEncoder3Connection,
            RobotSimulator.AngleEncoder4Connection,
            RobotSimulator.DriveEncoder1Connection,
            RobotSimulator.DriveEncoder2Connection,
            RobotSimulator.DriveEncoder3Connection,
            RobotSimulator.DriveEncoder4Connection,
            RobotSimulator.AbsoluteEncoder1Connection,
            RobotSimulator.AbsoluteEncoder2Connection,
            RobotSimulator.AbsoluteEncoder3Connection,
            RobotSimulator.AbsoluteEncoder4Connection,
            RobotSimulator.NavXConnection,
        };

    private final FauxbotActuatorConnection[] actuators =
        new FauxbotActuatorConnection[]
        {
            RobotSimulator.AngleMotor1Connection,
            RobotSimulator.AngleMotor2Connection,
            RobotSimulator.AngleMotor3Connection,
            RobotSimulator.AngleMotor4Connection,
            RobotSimulator.DriveMotor1Connection,
            RobotSimulator.DriveMotor2Connection,
            RobotSimulator.DriveMotor3Connection,
            RobotSimulator.DriveMotor4Connection
        };

    @SuppressWarnings("serial")
    private final Map<FauxbotSensorConnection, String> sensorNameMap = new HashMap<FauxbotSensorConnection, String>()
    {
        {
            this.put(RobotSimulator.AngleEncoder1Connection, "Angle encoder 1");
            this.put(RobotSimulator.AngleEncoder2Connection, "Angle encoder 2");
            this.put(RobotSimulator.AngleEncoder3Connection, "Angle encoder 3");
            this.put(RobotSimulator.AngleEncoder4Connection, "Angle encoder 4");
            this.put(RobotSimulator.DriveEncoder1Connection, "Drive encoder 1");
            this.put(RobotSimulator.DriveEncoder2Connection, "Drive encoder 2");
            this.put(RobotSimulator.DriveEncoder3Connection, "Drive encoder 3");
            this.put(RobotSimulator.DriveEncoder4Connection, "Drive encoder 4");
            this.put(RobotSimulator.AbsoluteEncoder1Connection, "Absolute encoder 1");
            this.put(RobotSimulator.AbsoluteEncoder2Connection, "Absolute encoder 2");
            this.put(RobotSimulator.AbsoluteEncoder3Connection, "Absolute encoder 3");
            this.put(RobotSimulator.AbsoluteEncoder4Connection, "Absolute encoder 4");
            this.put(RobotSimulator.NavXConnection, "NavX Yaw");
        }
    };

    @SuppressWarnings("serial")
    private final Map<FauxbotActuatorConnection, String> motorNameMap = new HashMap<FauxbotActuatorConnection, String>()
    {
        {
            this.put(RobotSimulator.AngleMotor1Connection, "Angle Motor 1");
            this.put(RobotSimulator.AngleMotor2Connection, "Angle Motor 2");
            this.put(RobotSimulator.AngleMotor3Connection, "Angle Motor 3");
            this.put(RobotSimulator.AngleMotor4Connection, "Angle Motor 4");
            this.put(RobotSimulator.DriveMotor1Connection, "Drive Motor 1");
            this.put(RobotSimulator.DriveMotor2Connection, "Drive Motor 2");
            this.put(RobotSimulator.DriveMotor3Connection, "Drive Motor 3");
            this.put(RobotSimulator.DriveMotor4Connection, "Drive Motor 4");
        }
    };

    private double prevTime;

    private double[] prevPosition;
    private double[] prevVelocity;
    private double[] prevAngle;

    @Inject
    public RobotSimulator()
    {
        this.prevTime = 0.0;

        this.prevPosition = new double[4];
        this.prevVelocity = new double[4];
        this.prevAngle = new double[4];
        for (int i = 0; i < 4; i++)
        {
            this.prevPosition[i] = 0.0;
            this.prevVelocity[i] = 0.0;
            this.prevAngle[i] = 0.0;
        }
    }

    @Override
    public FauxbotSensorConnection[] getSensors()
    {
        return this.sensors;
    }

    @Override
    public FauxbotActuatorConnection[] getActuators()
    {
        return this.actuators;
    }

    @Override
    public boolean getSensorTextBox(FauxbotSensorConnection connection)
    {
        if (connection == RobotSimulator.AngleEncoder1Connection ||
            connection == RobotSimulator.AngleEncoder2Connection ||
            connection == RobotSimulator.AngleEncoder3Connection ||
            connection == RobotSimulator.AngleEncoder4Connection ||
            connection == RobotSimulator.DriveEncoder1Connection ||
            connection == RobotSimulator.DriveEncoder2Connection ||
            connection == RobotSimulator.DriveEncoder3Connection ||
            connection == RobotSimulator.DriveEncoder4Connection ||
            connection == RobotSimulator.NavXConnection)
        {
            return true;
        }

        return false;
    }

    @Override
    public String getSensorName(FauxbotSensorConnection connection)
    {
        if (this.sensorNameMap.containsKey(connection))
        {
            return this.sensorNameMap.get(connection);
        }

        return "Sensor " + connection;
    }

    @Override
    public double getSensorMin(FauxbotSensorConnection connection)
    {
        if (connection == RobotSimulator.AngleEncoder1Connection ||
            connection == RobotSimulator.AngleEncoder2Connection ||
            connection == RobotSimulator.AngleEncoder3Connection ||
            connection == RobotSimulator.AngleEncoder4Connection)
        {
            return 0.0;
        }

        if (connection == RobotSimulator.DriveEncoder1Connection ||
            connection == RobotSimulator.DriveEncoder2Connection ||
            connection == RobotSimulator.DriveEncoder3Connection ||
            connection == RobotSimulator.DriveEncoder4Connection)
        {
            return -17000.0;
        }

        if (connection == RobotSimulator.AbsoluteEncoder1Connection ||
            connection == RobotSimulator.AbsoluteEncoder2Connection ||
            connection == RobotSimulator.AbsoluteEncoder3Connection ||
            connection == RobotSimulator.AbsoluteEncoder4Connection)
        {
            return 0.0;
        }

        return 0;
    }

    @Override
    public double getSensorMax(FauxbotSensorConnection connection)
    {
        if (connection == RobotSimulator.AngleEncoder1Connection ||
            connection == RobotSimulator.AngleEncoder2Connection ||
            connection == RobotSimulator.AngleEncoder3Connection ||
            connection == RobotSimulator.AngleEncoder4Connection)
        {
            return 36864.0;
        }

        if (connection == RobotSimulator.DriveEncoder1Connection ||
            connection == RobotSimulator.DriveEncoder2Connection ||
            connection == RobotSimulator.DriveEncoder3Connection ||
            connection == RobotSimulator.DriveEncoder4Connection)
        {
            return 17000.0;
        }

        if (connection == RobotSimulator.AbsoluteEncoder1Connection ||
            connection == RobotSimulator.AbsoluteEncoder2Connection ||
            connection == RobotSimulator.AbsoluteEncoder3Connection ||
            connection == RobotSimulator.AbsoluteEncoder4Connection)
        {
            return 359.9;
        }

        return 0;
    }

    @Override
    public String getActuatorName(FauxbotActuatorConnection connection)
    {
        if (this.motorNameMap.containsKey(connection))
        {
            return this.motorNameMap.get(connection);
        }

        return "Motor " + connection;
    }

    @Override
    public double getMotorMin(FauxbotActuatorConnection connection)
    {
        return -17000.0;
    }

    @Override
    public double getMotorMax(FauxbotActuatorConnection connection)
    {
        return 17000.0;
    }

    @Override
    public boolean shouldSimulatePID()
    {
        return false;
    }

    @Override
    public void update()
    {
        double currTime = Calendar.getInstance().getTime().getTime() / 1000.0;
        double dt = currTime - this.prevTime;
        for (int i = 0; i < 4; i++)
        {
            double currPosition = this.prevPosition[i];
            double currVelocity = this.prevVelocity[i];
            double currAngle = this.prevAngle[i];

            double driveMotorSetpoint = 0.0;
            FauxbotActuatorBase driveActuator = FauxbotActuatorManager.get(RobotSimulator.DriveMotors[i]);
            if (driveActuator != null && driveActuator instanceof FauxbotMotorBase)
            {
                FauxbotMotorBase motor = (FauxbotMotorBase)driveActuator;
                driveMotorSetpoint = motor.get();
            }

            currVelocity = driveMotorSetpoint;
            currPosition += (currVelocity * dt);

            this.prevPosition[i] = currPosition;
            this.prevVelocity[i] = currVelocity;

            FauxbotSensorBase driveSensor = FauxbotSensorManager.get(RobotSimulator.DriveEncoders[i]);
            if (driveSensor != null && driveSensor instanceof FauxbotEncoder)
            {
                FauxbotEncoder encoder = (FauxbotEncoder)driveSensor;
                encoder.set(this.prevPosition[i] + this.prevVelocity[i] * dt);
            }

            double angleMotorSetpoint = 0.0;
            FauxbotActuatorBase angleActuator = FauxbotActuatorManager.get(RobotSimulator.AngleMotors[i]);
            if (angleActuator != null && angleActuator instanceof FauxbotMotorBase)
            {
                FauxbotMotorBase motor = (FauxbotMotorBase)angleActuator;
                angleMotorSetpoint = motor.get();
            }

            currAngle = angleMotorSetpoint;

            this.prevAngle[i] = currAngle;

            FauxbotSensorBase angleSensor = FauxbotSensorManager.get(RobotSimulator.AngleEncoders[i]);
            if (angleSensor != null && angleSensor instanceof FauxbotEncoder)
            {
                FauxbotEncoder encoder = (FauxbotEncoder)angleSensor;
                encoder.set((int)this.prevAngle[i]);
            }
        }

        this.prevTime = currTime;
    }

    @Override
    public void draw(Canvas canvas)
    {
    }
}
