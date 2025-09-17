package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.RadahnChassis;
import org.firstinspires.ftc.teamcode.mechanisms.slidesArmSystem.OptimusSlidesArmSystem;

@TeleOp
public class SlidesTesting extends LinearOpMode {
    RadahnChassis chassis;
    OptimusSlidesArmSystem slidesArmSystem;
    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);
        slidesArmSystem = new OptimusSlidesArmSystem(gamepad1, telemetry, hardwareMap);

        while (opModeInInit()){
            slidesArmSystem.setPositions();

            telemetry.addLine("Waiting For Start");
            telemetry.update();
        }

        while (opModeIsActive()){
            chassis.robotCentricDrive();
            chassis.updatePose();

            slidesArmSystem.controllerInput();
            slidesArmSystem.setPositions();

            slidesArmSystem.setTelemetry();

//            telemetry.addData("Pose Estimate", chassis.getPose());
//            telemetry.addData("loop time", runtime.seconds()-previousTime);
            telemetry.update();

            previousTime = runtime.seconds();
        }
    }
}
