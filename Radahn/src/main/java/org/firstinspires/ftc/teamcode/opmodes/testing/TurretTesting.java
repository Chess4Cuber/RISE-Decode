package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.baseCode.CameraVision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.mechanisms.turretSystem.RadahnTurretSystem;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@TeleOp(name = "Turret Testing")
public class TurretTesting extends LinearOpMode {

    RadahnTurretSystem turretSystem;
    OpenCvCamera camera;
    AprilTagDetectionPipeline pipeline;

    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;

    static final double TAG_SIZE = 0.0762; // 3 inches in meters
    static final double FX = 578.272;
    static final double FY = 578.272;
    static final double CX = 402.145;
    static final double CY = 221.506;
    static final int CAMERA_WIDTH = 640;
    static final int CAMERA_HEIGHT = 480;

    boolean isBlueAlliance = true;
    boolean lastToggleY = false;

    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;

    @Override
    public void runOpMode() throws InterruptedException {

        // Combine DS and Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PID_Controller turretPID = new PID_Controller(0.005, 0.01, 0.7, 0.0001);
        turretPID.tolerance = 0.02;

        turretSystem = new RadahnTurretSystem(
                hardwareMap, telemetry, 1.0, turretPID, CAMERA_WIDTH, Math.toRadians(60)
        );

        pipeline = new AprilTagDetectionPipeline(TAG_SIZE, FX, FY, CX, CY);
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera", "Failed to open: " + errorCode);
                telemetry.update();
            }
        });

        while (opModeInInit()) {
            if ((gamepad1.y != lastToggleY) && gamepad1.y) {
                isBlueAlliance = !isBlueAlliance;
            }
            lastToggleY = gamepad1.y;

            telemetry.addLine("Waiting for start");
            telemetry.addData("Alliance", isBlueAlliance ? "BLUE" : "RED");
            telemetry.update();
        }

        while (opModeIsActive()) {
            int targetTagID = isBlueAlliance ? BLUE_GOAL_TAG_ID : RED_GOAL_TAG_ID;
            List<AprilTagDetection> detections = pipeline.getLatestDetections();

            turretSystem.update(detections, targetTagID);

            double manualPower = 0;
            if (gamepad1.left_bumper) manualPower = -0.2;
            else if (gamepad1.right_bumper) manualPower = 0.2;

            if (manualPower != 0) {
                turretSystem.getTurret().setPower(manualPower);
            }

            // Telemetry
            telemetry.addData("Alliance", isBlueAlliance ? "BLUE" : "RED");
            telemetry.addData("Loop Time", runtime.seconds() - previousTime);
            telemetry.addData("Encoder Ticks", turretSystem.getTurret().motors[0].getCurrPosTicks());
            telemetry.addData("Current Angle (rad)", turretSystem.getTurret().getCurrentAngle());
            telemetry.addData("PID Power", turretPID.PID_Power(
                    turretSystem.getTurret().getCurrentAngle(),
                    turretSystem.getTurret().getCurrentAngle() // current target for demonstration
            ));
            telemetry.addData("Manual Power", manualPower);
            telemetry.update();

            previousTime = runtime.seconds();
        }


        turretSystem.getTurret().setPower(0);
        if (camera != null) {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }

}