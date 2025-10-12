package org.firstinspires.ftc.baseCode.sensors.odometry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.math.Vector3D;

public class Odometry {
    double[] pose = new double[3];

    double x_pos = 0;
    double y_pos = 0;
    double heading = 0;
    double[] previousEncoderPos = {0,0,0};

    public Encoder leftEncoder, rightEncoder, middleEncoder;

    double[] encoderReadings = {0,0,0};

    // Array order convention: odoConstants = {CPR, wheelDia, TRACKWIDTH, FORWARD_OFFSET};
    double[] odoConstants;

    OdometryType odoType;

    public Odometry(String[] names, OdometryType odoType, double[] odoConstants, HardwareMap hardwareMap){
        switch (odoType){
            case THREE_WHEEL:
                leftEncoder = new Encoder(names[0], odoConstants[0], odoConstants[1], hardwareMap);
                rightEncoder = new Encoder(names[1], odoConstants[0], odoConstants[1], hardwareMap);
                middleEncoder = new Encoder(names[2], odoConstants[0], odoConstants[1], hardwareMap);
                break;
            case TWO_WHEEL:
                leftEncoder = new Encoder(names[0], odoConstants[0], odoConstants[1], hardwareMap);
                middleEncoder = new Encoder(names[2], odoConstants[0], odoConstants[1], hardwareMap);
                break;
        }

        this.odoType = odoType;
        this.odoConstants = odoConstants;
    }

    public void updatePose(){
        switch (odoType){
            case THREE_WHEEL:
                double deltaLeftEncoderPos = leftEncoder.getCurrPosInches() - previousEncoderPos[0];
                double deltaMiddleEncoderPos = middleEncoder.getCurrPosInches() - previousEncoderPos[2];
                double deltaRightEncoderPos = rightEncoder.getCurrPosInches() - previousEncoderPos[1];

                double phi = (deltaRightEncoderPos - deltaLeftEncoderPos)/odoConstants[2];
                double deltaMiddlePos = (deltaLeftEncoderPos + deltaRightEncoderPos)/2;
                double deltaPerpPos = deltaMiddleEncoderPos - odoConstants[3]*phi;

                double delta_x = deltaMiddlePos*Math.cos(heading) - deltaPerpPos*Math.sin(heading);
                double delta_y = deltaMiddlePos*Math.sin(heading) + deltaPerpPos*Math.cos(heading);

                x_pos += delta_x;
                y_pos += delta_y;
                heading += phi;

                encoderReadings[0] = leftEncoder.getCurrPosInches();
                encoderReadings[1] = rightEncoder.getCurrPosInches();
                encoderReadings[2] = middleEncoder.getCurrPosInches();

                pose[0] = x_pos;
                pose[1] = y_pos;
                pose[2] = Math.toDegrees(heading);

                previousEncoderPos[0] = leftEncoder.getCurrPosInches();
                previousEncoderPos[1] = rightEncoder.getCurrPosInches();
                previousEncoderPos[2] = middleEncoder.getCurrPosInches();

                break;

            case TWO_WHEEL:

                break;
        }


    }
    public double[] getPose() {
        return pose;
    }

    public Vector3D getPoseVector() {
        return new Vector3D(x_pos, y_pos, heading);
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
