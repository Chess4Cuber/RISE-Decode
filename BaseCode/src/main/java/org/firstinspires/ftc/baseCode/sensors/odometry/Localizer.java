package org.firstinspires.ftc.baseCode.sensors.odometry;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.baseCode.sensors.imu;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Localizer {
    private final DcMotorEx perp, par;
    private final imu IMU;
    private final double PARALLEL_OFFSET_IN, PERP_OFFSET_IN, TICKS_TO_INCHES;
    private double lastX, lastY, lastHead_rad;
    private Pose2D position;

    public Localizer(DcMotorEx perp, DcMotorEx par, imu IMU, double xOffsetIn, double yOffsetIn, double ticksToInches) {
        this.perp = perp;
        this.par = par;
        this.IMU = IMU;
        PARALLEL_OFFSET_IN = xOffsetIn;
        PERP_OFFSET_IN = yOffsetIn;
        TICKS_TO_INCHES = ticksToInches;
        position = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
    }

    public void update() {
        // Read poses
        double curX_in = par.getCurrentPosition() * TICKS_TO_INCHES;
        double curY_in = perp.getCurrentPosition() * TICKS_TO_INCHES;
        double curHead_rad = AngleUnit.normalizeRadians(Math.toRadians(IMU.Angle_FieldCentric()));

        // Deltas
        double dHead_rad = AngleUnit.normalizeRadians(curHead_rad - lastHead_rad);
        double dX_in = curX_in - lastX - PARALLEL_OFFSET_IN * dHead_rad;
        double dY_in = curY_in - lastY - PERP_OFFSET_IN * dHead_rad;

        // Rotate vector
        double cos = Math.cos(curHead_rad);
        double sin = Math.sin(curHead_rad);
        double xChange = dX_in * cos - dY_in * sin;
        double yChange = dX_in * sin + dY_in * cos;

        // Update last vals
        lastX = curX_in;
        lastY = curY_in;
        lastHead_rad = curHead_rad;

        // Accumulate position directly into the position object
        position = new Pose2D(
                DistanceUnit.INCH,
                position.getX(DistanceUnit.INCH) + xChange,
                position.getY(DistanceUnit.INCH) + yChange,
                AngleUnit.RADIANS,
                curHead_rad
        );
    }

    /**
     * @param x_in New x position of the robot in inches;
     * @param y_in New x position of the robot in inches;
     * @param heading_deg New heading of the robot in degrees;
     */
    public void setPosition(double x_in, double y_in, double heading_deg) {
        position = new Pose2D(DistanceUnit.INCH, x_in, y_in, AngleUnit.RADIANS, Math.toRadians(heading_deg));
        lastX = par.getCurrentPosition() * TICKS_TO_INCHES;
        lastY = perp.getCurrentPosition() * TICKS_TO_INCHES;
        lastHead_rad = AngleUnit.normalizeRadians(Math.toRadians(IMU.Angle_FieldCentric()));
    }

    public Pose2D getPosition() { return position; }
}