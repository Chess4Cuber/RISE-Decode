package org.firstinspires.ftc.teamcode.opmodes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.RadahnChassis;
import org.firstinspires.ftc.teamcode.mechanisms.intakeSystem.RadahnServoIntakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.linkageArmSystem.RadahnLinkageArmSystem;

@TeleOp
public class LinkageTestingRadahn extends LinearOpMode {
    RadahnChassis chassis;

    RadahnLinkageArmSystem linkageArmSystem;
    RadahnServoIntakeSystem intakeSystem;
    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);

        linkageArmSystem = new RadahnLinkageArmSystem(gamepad1, telemetry, hardwareMap);
        intakeSystem = new RadahnServoIntakeSystem(gamepad1, hardwareMap);

        while (opModeInInit()){
            linkageArmSystem.setPositions();
            intakeSystem.setPositions();

            telemetry.addLine("Waiting For Start");
            telemetry.update();
        }

        while (opModeIsActive()){
            chassis.robotCentricDrive();
            chassis.updatePose();

            linkageArmSystem.controllerInput();
            linkageArmSystem.setPositions();

            intakeSystem.controllerInput();
            intakeSystem.setPositions();

            telemetry.addData("Pose Estimate", chassis.getPose());
            telemetry.addData("loop time", runtime.seconds()-previousTime);
            telemetry.update();

            previousTime = runtime.seconds();
        }
    }
}
