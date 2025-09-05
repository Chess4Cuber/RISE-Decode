package org.firstinspires.ftc.teamcode.opmodes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.OptimusChassis;
import org.firstinspires.ftc.teamcode.mechanisms.OptimusClaw;
import org.firstinspires.ftc.teamcode.mechanisms.intakeSystem.OptimusIntakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.linkageArmSystem.OptimusLinkageArmSystem;
import org.firstinspires.ftc.teamcode.mechanisms.slidesArmSystem.OptimusSlidesArmSystem;

@TeleOp
public class OptimusTeleOp extends LinearOpMode {
    OptimusChassis chassis;
    OptimusSlidesArmSystem slidesArmSystem;
    OptimusClaw claw;
    OptimusLinkageArmSystem linkageArmSystem;
    OptimusIntakeSystem intakeSystem;
    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new OptimusChassis(gamepad1, telemetry, hardwareMap);
        slidesArmSystem = new OptimusSlidesArmSystem(gamepad1, telemetry, hardwareMap);
        linkageArmSystem = new OptimusLinkageArmSystem(gamepad1, telemetry, hardwareMap);
        intakeSystem = new OptimusIntakeSystem(gamepad1, hardwareMap);
        claw = new OptimusClaw(gamepad1, hardwareMap);


        while (opModeInInit()){
            slidesArmSystem.setPositions();
            claw.openClaw();
            linkageArmSystem.setPositions();
            //intakeSystem.setPositions();
            telemetry.addLine("Waiting For Start");
            telemetry.update();
        }

        while (opModeIsActive()){
            chassis.robotCentricDrive();
            chassis.updatePose();

            slidesArmSystem.controllerInput();
            slidesArmSystem.setPositions();

            linkageArmSystem.controllerInput();
            linkageArmSystem.setPositions();

            intakeSystem.controllerInput();
            intakeSystem.setPositions();

            claw.toggleClaw();

            telemetry.addData("Pose Estimate", chassis.getPose());
            telemetry.addData("loop time", runtime.seconds()-previousTime);
            telemetry.update();

            previousTime = runtime.seconds();
        }
    }
}
