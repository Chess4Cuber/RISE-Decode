package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.baseCode.math.Vector3D;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnChassis;


@Autonomous
public class PID_PointFollowerTest extends LinearOpMode {

    RadahnChassis chassis;
    Vector3D point = new Vector3D(0, 0, 35);

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            chassis.goToPosePID(point);
            chassis.updatePose();


            telemetry.addData("Pose Estimate X:", chassis.getPose()[0]);
            telemetry.addData("Pose Estimate Y:", chassis.getPose()[1]);
            telemetry.addData("Pose Estimate Angle:", chassis.getPose()[2]);

            telemetry.addData("HeadingPID error", chassis.HeadingPID.error);
            telemetry.addData("TranslationalX error", chassis.TranslationalPID_X.error);
            telemetry.addData("TranslationalY error", chassis.TranslationalPID_Y.error);
            telemetry.addData("Distance to Pose", point.findDistance(chassis.getPoseVector()));

            telemetry.addData("X PID Proportion", chassis.TranslationalPID_X.P);
            telemetry.addData("Y PID Proportion", chassis.TranslationalPID_Y.P);
            telemetry.addData("Heading PID Proportion", chassis.HeadingPID.P);

            telemetry.addData("TranslationalX PID Derivative", chassis.TranslationalPID_X.D);
            telemetry.addData("TranslationalY PID Derivative", chassis.TranslationalPID_Y.D);

            telemetry.addData("X PID Integral", chassis.TranslationalPID_X.I);
            telemetry.addData("Y PID Integral", chassis.TranslationalPID_Y.I);
            telemetry.addData("Heading PID Integral", chassis.HeadingPID.I);

            telemetry.update();
        }
    }
}