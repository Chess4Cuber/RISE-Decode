package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.RadahnChassis;
import org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem.RadahnMotorIntakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.motorOuttake.RadahnMotorOuttakeSystem;

@TeleOp
public class MotorIntakeTesting extends LinearOpMode {
    RadahnChassis chassis;

    //OptimusLinkageArmSystem linkageArmSystem;
    RadahnMotorIntakeSystem motorIntakeSystem;
    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        motorIntakeSystem = new RadahnMotorIntakeSystem(gamepad1, telemetry, hardwareMap);
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);

        while (opModeInInit()){

            telemetry.addLine("Waiting For Start");
            telemetry.update();
        }

        while (opModeIsActive()){

            motorIntakeSystem.controllerInput();
            motorIntakeSystem.setPositions();

            chassis.robotCentricDrive();
            chassis.updatePose();

            telemetry.addData("Pose Estimate", chassis.getPose());
            telemetry.addData("loop time", runtime.seconds()-previousTime);
            telemetry.update();

            previousTime = runtime.seconds();
        }
    }
}
