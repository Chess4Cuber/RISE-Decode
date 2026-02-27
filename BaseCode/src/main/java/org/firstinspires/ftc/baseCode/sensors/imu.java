package org.firstinspires.ftc.baseCode.sensors;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class imu {
    public IMU imu;

    private double startingYaw = 0;

    // Default constructor — assumes Control Hub is flat, USB ports facing forward
    public imu(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();

        // Record starting yaw so Angle_FieldCentric() returns relative heading
        startingYaw = getRawYawRadians();
    }

    // Constructor that lets you specify Control Hub mounting orientation
    public imu(RevHubOrientationOnRobot.LogoFacingDirection logoDir,
               RevHubOrientationOnRobot.UsbFacingDirection usbDir,
               HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(logoDir, usbDir)
        );

        imu.initialize(parameters);
        imu.resetYaw();

        startingYaw = getRawYawRadians();
    }

    // Returns raw yaw in radians from IMU
    private double getRawYawRadians() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.RADIANS);
    }

    // Returns heading in radians relative to starting position
    // Negative sign keeps same convention as the old BNO055 implementation
    public double Angle_FieldCentric() {
        double currentYaw = getRawYawRadians();
        double relativeYaw = currentYaw - startingYaw;

        // Wrap to [-pi, pi]
        while (relativeYaw >  Math.PI) relativeYaw -= 2 * Math.PI;
        while (relativeYaw < -Math.PI) relativeYaw += 2 * Math.PI;

        return -relativeYaw;
    }

    public double getFirstAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getSecondAngle() {
        return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS);
    }

    public double getThirdAngle() {
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS);
    }

    public double getXAngularVelocity() {
        AngularVelocity velocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        return velocity.xRotationRate;
    }

    public double getYAngularVelocity() {
        AngularVelocity velocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        return velocity.yRotationRate;
    }

    public double getZAngularVelocity() {
        AngularVelocity velocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        return velocity.zRotationRate;
    }
}