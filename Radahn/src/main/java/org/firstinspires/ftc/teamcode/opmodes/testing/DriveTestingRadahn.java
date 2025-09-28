package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.RadahnChassis;

@TeleOp
public class DriveTestingRadahn extends LinearOpMode {
    RadahnChassis chassis;
    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);
        telemetry.addLine("Waiting For Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            chassis.robotCentricDrive();
            chassis.updatePose();

            telemetry.addData("Pose Estimate X:", chassis.getPose()[0]);
            telemetry.addData("Pose Estimate Y:", chassis.getPose()[1]);
            telemetry.addData("Pose Estimate Angle:", chassis.getPose()[2]);

            telemetry.addData("Encoder Reading Left:", chassis.getEncoderReadings()[0]);
            telemetry.addData("Encoder Reading Right:", chassis.getEncoderReadings()[1]);
            telemetry.addData("Encoder Reading Middle:", chassis.getEncoderReadings()[2]);

            telemetry.addData("loop time", runtime.seconds()-previousTime);
            telemetry.update();

            previousTime = runtime.seconds();
        }
    }
}
