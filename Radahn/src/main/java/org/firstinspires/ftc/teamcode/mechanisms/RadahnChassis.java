package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.baseCode.hardware.chassis.MecanumChassis;
import org.firstinspires.ftc.baseCode.sensors.odometry.OdometryType;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnChassis extends MecanumChassis {

    double CPR = 2000;
    double wheelDia = 1.89;
    double trackwidth = 9.5;
    double forwardOffset = 5.25;

    public PID_Controller TranslationalPID_X;
    public PID_Controller TranslationalPID_Y;
    public PID_Controller HeadingPID;
    public RadahnChassis(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super(new String[]{"fLeft", "fRight", "bRight", "bLeft"}, gamepad1, telemetry, hardwareMap);

        setOdometry(new String[]{"fLeft", "fRight", "bRight"}, OdometryType.THREE_WHEEL,
                new double[]{CPR, wheelDia, trackwidth, forwardOffset}, hardwareMap, -.88, .88, -1);

        TranslationalPID_X = new PID_Controller(0.05);
        TranslationalPID_Y = new PID_Controller(0.05);
        HeadingPID = new PID_Controller(0.02);

        setPID_Controller(TranslationalPID_X, TranslationalPID_Y, HeadingPID);

        frontLeft.setDirectionForward();
        frontRight.setDirectionReverse();
        backRight.setDirectionReverse();
        backLeft.setDirectionForward();
    }


}
