package org.firstinspires.ftc.baseCode.sensors.odometry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.math.Vector3D;

public class Odometry {
    public double[] pose = new double[3];  // made public so IMU can override pose[2]

    double x_pos = 0;
    double y_pos = 0;
    public double heading = 0;             // made public so IMU can override heading

    double x_offset = 0;
    double y_offset = 0;
    double head_offset = 0;

    double[] previousEncoderPos = {0, 0, 0};

    public Encoder leftEncoder, rightEncoder, middleEncoder;

    double[] encoderReadings = {0, 0, 0};

    // Array order convention: odoConstants = {CPR, wheelDia, TRACKWIDTH, FORWARD_OFFSET}
    // For TWO_WHEEL: TRACKWIDTH and FORWARD_OFFSET are unused (heading comes from IMU)
    double[] odoConstants;

    OdometryType odoType;

    // names[0] = parallel encoder (forward/back)
    // names[1] = right encoder    (THREE_WHEEL only)
    // names[2] = strafe encoder   (perpendicular)
    public Odometry(String[] names, OdometryType odoType, double[] odoConstants, HardwareMap hardwareMap, double xOff, double yOff, double headOff){
        switch (odoType){
            case THREE_WHEEL:
                leftEncoder   = new Encoder(names[0], odoConstants[0], odoConstants[1], hardwareMap);
                rightEncoder  = new Encoder(names[1], odoConstants[0], odoConstants[1], hardwareMap);
                middleEncoder = new Encoder(names[2], odoConstants[0], odoConstants[1], hardwareMap);
                x_offset    = xOff;
                y_offset    = yOff;
                head_offset = headOff;
                break;

            case TWO_WHEEL:
                // leftEncoder   = parallel wheel (measures forward/back movement)
                // middleEncoder = strafe wheel   (measures lateral movement)
                // rightEncoder is NOT used — heading comes entirely from the IMU
                leftEncoder   = new Encoder(names[0], odoConstants[0], odoConstants[1], hardwareMap);
                middleEncoder = new Encoder(names[2], odoConstants[0], odoConstants[1], hardwareMap);
                x_offset    = xOff;
                y_offset    = yOff;
                head_offset = headOff;
                break;
        }

        this.odoType      = odoType;
        this.odoConstants = odoConstants;
    }

    public void updatePose() {
        switch (odoType) {
            case THREE_WHEEL: {
                double deltaLeftEncoderPos   = leftEncoder.getCurrPosInches()   - previousEncoderPos[0];
                double deltaRightEncoderPos  = rightEncoder.getCurrPosInches()  - previousEncoderPos[1];
                double deltaMiddleEncoderPos = middleEncoder.getCurrPosInches() - previousEncoderPos[2];

                // phi used for X/Y integration and heading if no IMU
                double phi = (deltaRightEncoderPos - deltaLeftEncoderPos) / odoConstants[2];

                double deltaParallel = (deltaLeftEncoderPos + deltaRightEncoderPos) / 2.0;
                double deltaPerp     = deltaMiddleEncoderPos - (odoConstants[3] * phi);

                double deltaX = deltaParallel * Math.cos(heading) - deltaPerp * Math.sin(heading);
                double deltaY = deltaParallel * Math.sin(heading) + deltaPerp * Math.cos(heading);

                x_pos += deltaX;
                y_pos += deltaY;

                // IMU overrides this in MecanumChassis.updatePose() if active
                heading += phi;
                heading = Math.atan2(Math.sin(heading), Math.cos(heading));

                previousEncoderPos[0] = leftEncoder.getCurrPosInches();
                previousEncoderPos[1] = rightEncoder.getCurrPosInches();
                previousEncoderPos[2] = middleEncoder.getCurrPosInches();

                encoderReadings[0] = previousEncoderPos[0];
                encoderReadings[1] = previousEncoderPos[1];
                encoderReadings[2] = previousEncoderPos[2];

                pose[0] = x_pos * x_offset;
                pose[1] = y_pos * y_offset;
                pose[2] = Math.toDegrees(heading) * head_offset;
                break;
            }

            case TWO_WHEEL: {
                // heading is always set externally by the IMU via MecanumChassis.updatePose()
                // before this method is called. No phi term — IMU owns heading entirely.

                double deltaParallelEncoderPos = leftEncoder.getCurrPosInches()   - previousEncoderPos[0];
                double deltaStrafeEncoderPos   = middleEncoder.getCurrPosInches() - previousEncoderPos[2];

                // Integrate X/Y using the current IMU heading (in radians)
                double deltaX = deltaParallelEncoderPos * Math.cos(heading) - deltaStrafeEncoderPos * Math.sin(heading);
                double deltaY = deltaParallelEncoderPos * Math.sin(heading) + deltaStrafeEncoderPos * Math.cos(heading);

                x_pos += deltaX;
                y_pos += deltaY;

                // heading is NOT modified here — IMU owns it entirely

                // index 1 skipped — no right encoder in TWO_WHEEL mode
                previousEncoderPos[0] = leftEncoder.getCurrPosInches();
                previousEncoderPos[2] = middleEncoder.getCurrPosInches();

                encoderReadings[0] = previousEncoderPos[0];
                encoderReadings[2] = previousEncoderPos[2];

                pose[0] = x_pos * x_offset;
                pose[1] = y_pos * y_offset;
                pose[2] = Math.toDegrees(heading) * head_offset;
                break;
            }
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

    public void setPose(double x, double y, double headingDeg) {
        this.x_pos   = x;
        this.y_pos   = y;
        this.heading = Math.toRadians(headingDeg);

        pose[0] = x;
        pose[1] = y;
        pose[2] = headingDeg;
    }

    public void resetEncoderDeltas() {
        previousEncoderPos[0] = leftEncoder.getCurrPosInches();
        // Only reset right encoder if it exists (THREE_WHEEL mode only)
        if (rightEncoder != null) {
            previousEncoderPos[1] = rightEncoder.getCurrPosInches();
        }
        previousEncoderPos[2] = middleEncoder.getCurrPosInches();
    }
}