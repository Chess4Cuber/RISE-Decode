package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.OptimusChassis;
import org.firstinspires.ftc.teamcode.mechanisms.OptimusClaw;
import org.firstinspires.ftc.teamcode.mechanisms.intakeSystem.OptimusIntakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.linkageArmSystem.OptimusLinkage;
import org.firstinspires.ftc.teamcode.mechanisms.linkageArmSystem.OptimusLinkageArmSystem;
import org.firstinspires.ftc.teamcode.mechanisms.slidesArmSystem.OptimusSlidesArmSystem;

@TeleOp
public class ServoTesting extends LinearOpMode {
    //OptimusChassis chassis;

    //OptimusLinkageArmSystem linkageArmSystem;
    OptimusIntakeSystem intakeSystem;
    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        intakeSystem = new OptimusIntakeSystem(gamepad1, hardwareMap);

        while (opModeInInit()){
            intakeSystem.setPositions();

            telemetry.addLine("Waiting For Start");
            telemetry.update();
        }

        while (opModeIsActive()){

            intakeSystem.controllerInput();
            intakeSystem.setPositions();

            //telemetry.addData("Pose Estimate", chassis.getPose());
            telemetry.addData("loop time", runtime.seconds()-previousTime);
            telemetry.update();

            previousTime = runtime.seconds();
        }
    }
}
