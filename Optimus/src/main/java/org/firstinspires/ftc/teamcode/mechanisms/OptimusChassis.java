package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.baseCode.hardware.chassis.MecanumChassis;
import org.firstinspires.ftc.baseCode.sensors.odometry.OdometryType;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OptimusChassis extends MecanumChassis {

    double CPR = 537.7;
    double wheelDia = 3.7795;
    double trackwidth = 13;
    double forwardOffset = 7;

    PID_Controller TranslationalPID_X;
    PID_Controller TranslationalPID_Y;
    PID_Controller HeadingPID;
    public OptimusChassis(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super(new String[]{"fLeft", "fRight", "bRight", "bLeft"}, gamepad1, telemetry, hardwareMap);

        setOdometry(new String[]{"fLeft", "fRight", "bRight"}, OdometryType.THREE_WHEEL,
                new double[]{CPR, wheelDia, trackwidth, forwardOffset}, hardwareMap);

        TranslationalPID_X = new PID_Controller(0);
        TranslationalPID_Y = new PID_Controller(0);
        HeadingPID = new PID_Controller(0);

        setPID_Controller(TranslationalPID_X, TranslationalPID_Y, HeadingPID);

        frontLeft.setDirectionReverse();
        frontRight.setDirectionForward();
        backRight.setDirectionForward();
        backLeft.setDirectionReverse();
    }
}
