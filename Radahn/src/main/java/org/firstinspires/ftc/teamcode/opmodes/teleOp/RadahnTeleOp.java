package org.firstinspires.ftc.teamcode.opmodes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.RadahnChassis;
import org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem.RadahnMotorIntakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.motorOuttakeSystem.RadahnMotorOuttakeSystem;

@TeleOp
public class RadahnTeleOp extends LinearOpMode {
    RadahnChassis chassis;
    //RadahnServoIntakeSystem intakeSystem;
    RadahnMotorOuttakeSystem motorOuttakeSystem;

    RadahnMotorIntakeSystem motorIntakeSystem;

    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        motorOuttakeSystem = new RadahnMotorOuttakeSystem(gamepad1, telemetry, hardwareMap);
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);
        motorIntakeSystem = new RadahnMotorIntakeSystem(gamepad1, telemetry, hardwareMap);


        while (opModeInInit()){
            telemetry.addLine("Waiting For Start");
            telemetry.update();
        }

        while (opModeIsActive()){
            chassis.robotCentricDrive();
            chassis.updatePose();

            motorOuttakeSystem.controllerInput();
            motorOuttakeSystem.setPositions();

            motorIntakeSystem.controllerInput();
            motorIntakeSystem.setPositions();


 //           telemetry.addData("Pose Estimate", chassis.getPose());
            telemetry.addData("loop time", runtime.seconds()-previousTime);
            telemetry.update();

            previousTime = runtime.seconds();
        }
    }
}
