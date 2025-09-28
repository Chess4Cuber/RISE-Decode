package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.RadahnChassis;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnSpindexerSystem.RadahnSpindexerSystem;
import org.firstinspires.ftc.teamcode.mechanisms.intakeSystem.RadahnServoIntakeSystem;

@TeleOp
public class SpindexerTestingRadahn extends LinearOpMode {
    RadahnChassis chassis;
    RadahnSpindexerSystem spindexerSystem;
    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);
        spindexerSystem = new RadahnSpindexerSystem(gamepad1, telemetry, hardwareMap);

        while (opModeInInit()){
            spindexerSystem.setPositions();

            telemetry.addLine("Waiting For Start");
            telemetry.update();
        }

        while (opModeIsActive()){
            chassis.robotCentricDrive();
            chassis.updatePose();

            spindexerSystem.controllerInput();
            spindexerSystem.setPositions();

//            telemetry.addData("Pose Estimate", chassis.getPose());
            telemetry.addData("loop time", runtime.seconds()-previousTime);
            telemetry.update();

            previousTime = runtime.seconds();
        }
    }
}
