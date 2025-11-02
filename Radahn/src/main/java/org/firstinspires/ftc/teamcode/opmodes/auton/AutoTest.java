package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.baseCode.math.Vector3D;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnChassis;

@Autonomous(name = "AutoTestRadahn_OneLoop", group = "Testing")
public class AutoTest extends LinearOpMode {

    RadahnChassis chassis;

    enum AutoStage { FORWARD, TURN, BACK, DONE }

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        AutoStage stage = AutoStage.FORWARD;

        // Define target positions
        Vector3D forward = new Vector3D(24, 0, 0);
        Vector3D turn = new Vector3D(24, 0, Math.toRadians(180));
        Vector3D back = new Vector3D(0, 0, Math.toRadians(180));

        while (opModeIsActive() && stage != AutoStage.DONE) {
            chassis.updatePose();

            switch (stage) {
                case FORWARD:
                    chassis.goToPosePID(forward);
                    if (Math.abs(chassis.getPose()[0] - forward.A) < 1.0 &&
                            Math.abs(chassis.getPose()[1] - forward.B) < 1.0) {
                        stage = AutoStage.TURN;
                    }
                    break;

                case TURN:
                    chassis.goToPosePID(turn);
                    if (Math.abs(chassis.getPose()[2] - Math.toDegrees(turn.C)) < 5.0) {
                        stage = AutoStage.BACK;
                    }
                    break;

                case BACK:
                    chassis.goToPosePID(back);
                    if (Math.abs(chassis.getPose()[0] - back.A) < 1.0 &&
                            Math.abs(chassis.getPose()[1] - back.B) < 1.0) {
                        stage = AutoStage.DONE;
                    }
                    break;
            }

            telemetry.addData("Stage", stage);
            telemetry.addData("Pose", chassis.getPoseVector());
            telemetry.update();
        }

        chassis.setPower(0);
        telemetry.addLine("Auto Complete!");
        telemetry.update();
    }
}
