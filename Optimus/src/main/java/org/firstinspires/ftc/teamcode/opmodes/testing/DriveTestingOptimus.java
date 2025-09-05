package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.OptimusChassis;

@TeleOp
public class DriveTestingOptimus extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        OptimusChassis chassis = new OptimusChassis(gamepad1, telemetry, hardwareMap);
        telemetry.addLine("Waiting For Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            chassis.robotCentricDrive();
            chassis.updatePose();

            telemetry.addData("Pose Estimate", chassis.getPose());
            telemetry.addData("loop time", runtime.seconds()-previousTime);
            telemetry.update();

            previousTime = runtime.seconds();
        }
    }
}
