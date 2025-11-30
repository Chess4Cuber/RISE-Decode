package org.firstinspires.ftc.baseCode.sensors.odometry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.math.Vector3D;

public class Odometry {
    double[] pose = new double[3];

    double x_pos = 0;
    double y_pos = 0;
    double heading = 0;

    double x_offset = 0;
    double y_offset = 0;
    double head_offset = 0;



    double[] previousEncoderPos = {0,0,0};

    public Encoder leftEncoder, rightEncoder, middleEncoder;

    double[] encoderReadings = {0,0,0};

    // Array order convention: odoConstants = {CPR, wheelDia, TRACKWIDTH, FORWARD_OFFSET};
    double[] odoConstants;

    OdometryType odoType;

    public Odometry(String[] names, OdometryType odoType, double[] odoConstants, HardwareMap hardwareMap, double xOff, double yOff, double headOff){
        switch (odoType){
            case THREE_WHEEL:
                leftEncoder = new Encoder(names[0], odoConstants[0], odoConstants[1], hardwareMap);
                rightEncoder = new Encoder(names[1], odoConstants[0], odoConstants[1], hardwareMap);
                middleEncoder = new Encoder(names[2], odoConstants[0], odoConstants[1], hardwareMap);
                x_offset = xOff;
                y_offset = yOff;
                head_offset = headOff;

                break;
            case TWO_WHEEL:
                leftEncoder = new Encoder(names[0], odoConstants[0], odoConstants[1], hardwareMap);
                middleEncoder = new Encoder(names[2], odoConstants[0], odoConstants[1], hardwareMap);
                break;
        }

        this.odoType = odoType;
        this.odoConstants = odoConstants;
    }

    public void updatePose() {
        switch (odoType) {
            case THREE_WHEEL:
                // Calculate encoder deltas in inches
                double deltaLeftEncoderPos = leftEncoder.getCurrPosInches() - previousEncoderPos[0];
                double deltaRightEncoderPos = rightEncoder.getCurrPosInches() - previousEncoderPos[1];
                double deltaMiddleEncoderPos = middleEncoder.getCurrPosInches() - previousEncoderPos[2];

                // Change in robot orientation (radians)
                double phi = (deltaRightEncoderPos - deltaLeftEncoderPos) / odoConstants[2];

                // Forward and lateral displacement in robot coordinates
                double deltaParallel = (deltaLeftEncoderPos + deltaRightEncoderPos) / 2.0;
                double deltaPerp = deltaMiddleEncoderPos - (odoConstants[3] * phi);


                double deltaX = deltaParallel * Math.cos(heading) - deltaPerp * Math.sin(heading);
                double deltaY = deltaParallel * Math.sin(heading) + deltaPerp * Math.cos(heading);

                // Update pose
                x_pos += deltaX;
                y_pos += deltaY;
                heading += phi;

                heading = Math.atan2(Math.sin(heading), Math.cos(heading));

                // Store current encoder positions for next loop
                previousEncoderPos[0] = leftEncoder.getCurrPosInches();
                previousEncoderPos[1] = rightEncoder.getCurrPosInches();
                previousEncoderPos[2] = middleEncoder.getCurrPosInches();

                // Update output arrays
                encoderReadings[0] = previousEncoderPos[0];
                encoderReadings[1] = previousEncoderPos[1];
                encoderReadings[2] = previousEncoderPos[2];

                pose[0] = x_pos * x_offset;
                pose[1] = y_pos * y_offset;
                pose[2] = Math.toDegrees(heading) * head_offset;
                break;

            case TWO_WHEEL:
                // TODO: implement later if needed
                break;
        }
    }

    public double[] getPose() {
        return pose;
    }

    public Vector3D getPoseVector() {
        return new Vector3D(pose[0], pose[1], pose[2]);
    }

    public double getX() {
        return x_pos;
    }
    public double getY() {
        return y_pos;
    }
    public double getHeading() {
        return heading;
    }

    public double[] getEncoderReadings() {
        return encoderReadings;
    }
}