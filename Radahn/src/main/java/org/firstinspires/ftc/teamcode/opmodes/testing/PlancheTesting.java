package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.RadahnChassis;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnPlanche.RadahnSlidesSystem;
import org.firstinspires.ftc.teamcode.mechanisms.otherMechanisms.slidesArmSystem.OptimusSlidesArmSystem;

@TeleOp
public class PlancheTesting extends LinearOpMode {
    RadahnChassis chassis;
    RadahnSlidesSystem slidesSystem;
    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);
        slidesSystem = new RadahnSlidesSystem(gamepad1, telemetry, hardwareMap);

        while (opModeInInit()){
            slidesSystem.setPositions();

            telemetry.addLine("Waiting For Start");
            telemetry.update();
        }

        while (opModeIsActive()){
            chassis.robotCentricDrive();
            chassis.updatePose();

            slidesSystem.controllerInput();
            slidesSystem.setPositions();

            slidesSystem.setTelemetry();

//            telemetry.addData("Pose Estimate", chassis.getPose());
//            telemetry.addData("loop time", runtime.seconds()-previousTime);
            telemetry.update();

            previousTime = runtime.seconds();
        }
    }
}
