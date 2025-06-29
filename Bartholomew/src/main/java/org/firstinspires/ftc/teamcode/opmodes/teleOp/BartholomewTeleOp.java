package org.firstinspires.ftc.teamcode.opmodes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.BartholomewChassis;
import org.firstinspires.ftc.teamcode.mechanisms.BartholomewClaw;
import org.firstinspires.ftc.teamcode.mechanisms.slidesArmSystem.SlidesArmSystem;

@TeleOp
public class BartholomewTeleOp extends LinearOpMode {
    BartholomewChassis chassis;
    SlidesArmSystem slidesArmSystem;
    BartholomewClaw claw;
    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new BartholomewChassis(gamepad1, telemetry, hardwareMap);
        slidesArmSystem = new SlidesArmSystem(gamepad1, telemetry, hardwareMap);
        claw = new BartholomewClaw(gamepad1, hardwareMap);

        while (opModeInInit()){
            slidesArmSystem.setPositions();
            claw.openClaw();
            telemetry.addLine("Waiting For Start");
            telemetry.update();
        }

        while (opModeIsActive()){
            chassis.robotCentricDrive();
            chassis.updatePose();

            slidesArmSystem.controllerInput();
            slidesArmSystem.setPositions();

            claw.toggleClaw();

            telemetry.addData("Pose Estimate", chassis.getPose());
            telemetry.addData("loop time", runtime.seconds()-previousTime);
            telemetry.update();

            previousTime = runtime.seconds();
        }
    }
}
