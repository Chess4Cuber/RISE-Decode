package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.baseCode.hardware.claws.SingleServoClaw;
import org.firstinspires.ftc.baseCode.math.Vector3D;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnChassis;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnColorSensor;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnTransfer.RadahnGate;
import org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem.RadahnHoodedOuttakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem.MotorIntakeStates;
import org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem.RadahnMotorIntakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.turretSystem.RadahnTurretSystem;
import org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem.TurretHoodStates;
import org.firstinspires.ftc.teamcode.mechanisms.turretSystem.TurretStates;

@Autonomous
public class Nine_Ball_Blue_FAR extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    public enum AutoStep{
        MOVE_TURN_TURRET,
        SHOOTPRE,
        FIRST_LINE,
        FIRST_LINE2,
        BACK_FIRST,
        SHOOT_FIRST,
        SECOND_LINE,
        BACK_SECOND,
        SHOOT_SECOND,
        THIRD_LINE,
        THIRD_LINE2,
        BACK_THIRD,
        SHOOT_THIRD,
        PARK
    }

    RadahnChassis chassis;
    RadahnMotorIntakeSystem intake;
    RadahnHoodedOuttakeSystem hoodedOuttakeSystem;
    RadahnTurretSystem turret;
    RadahnGate gate;
    RadahnColorSensor colorSensors;

    AutoStep parkingStep;

    Limelight3A limelight;

    // Camera geometry constants
    final double CAMERA_HEIGHT = 10.0;     // inches
    final double TARGET_HEIGHT = 24.0;     // inches
    final double CAMERA_ANGLE = Math.toRadians(30.0); // radians

    Vector3D poseVector = new Vector3D(0,0,0);
    Vector3D targetPose = new Vector3D(0, 0, 0);

    double tolerance = 3.75;

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);
        intake = new RadahnMotorIntakeSystem(gamepad1, telemetry, hardwareMap);
        hoodedOuttakeSystem = new RadahnHoodedOuttakeSystem(gamepad1, telemetry, hardwareMap);
        turret = new RadahnTurretSystem(gamepad1, telemetry, hardwareMap);
        gate = new RadahnGate(gamepad1, hardwareMap);

        colorSensors = new RadahnColorSensor(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        parkingStep = AutoStep.MOVE_TURN_TURRET;

        while (opModeInInit()){
            intake.setMotorIntakeState(MotorIntakeStates.RESTING);
            gate.openClaw();

            telemetry.update();
        }

        while (opModeIsActive()){

            double tagDistanceInches = 0;
            double tx = 0;
            boolean tagVisible = false;

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                tagVisible = true;

                tx = result.getTx();

                double tyRadians = Math.toRadians(result.getTy());
                tagDistanceInches = (TARGET_HEIGHT - CAMERA_HEIGHT) /
                        Math.tan(CAMERA_ANGLE + tyRadians);
            }

            chassis.updatePose();
            intake.setPositions();

            hoodedOuttakeSystem.updateDistance(tagDistanceInches, tagVisible);
            hoodedOuttakeSystem.update();

            turret.updateLimelight(tx, tagVisible);
            turret.update();

            gate.setPosition();

            autonBlue();

            chassis.goToPosePID(targetPose);
            poseVector.set(chassis.odo.getX(), chassis.odo.getY(), chassis.getPose()[2]);

            Telemetry();

            telemetry.update();
        }
    }

    public void autonBlue(){
        switch(parkingStep){
            case  MOVE_TURN_TURRET:
                targetPose.set(0, 6.65, 0);
                //turret.setState(TurretStates.MANUAL);
                //turret.setTargetAngle(-25.0);

                if (targetPose.findDistance(poseVector) < tolerance ){
                    hoodedOuttakeSystem.setMotorOuttakeState(TurretHoodStates.OUTTAKING);
                    intake.setMotorIntakeState(MotorIntakeStates.INTAKING);
                    turret.setState(TurretStates.AUTO_AIM);

                    parkingStep = AutoStep.SHOOTPRE;
                    runtime.reset();
                }

                break;

            case SHOOTPRE:
                gate.setClawState(SingleServoClaw.ClawState.OPEN);

                if (runtime.seconds() > 1) {
                    gate.setClawState(SingleServoClaw.ClawState.CLOSED);
                    intake.setMotorIntakeState(MotorIntakeStates.RESTING);

                    parkingStep = AutoStep.FIRST_LINE;
                    runtime.reset();
                }
                break;

            case FIRST_LINE:
                targetPose.set(-7.26, 24.31, 0);

                if(targetPose.findDistance(poseVector) < tolerance){
                    intake.setMotorIntakeState(MotorIntakeStates.INTAKING);

                    parkingStep = AutoStep.FIRST_LINE2;
                    runtime.reset();
                }
                break;

            case FIRST_LINE2:
                targetPose.set(-27.55, 24.31, 0);

                if(targetPose.findDistance(poseVector) < tolerance){
                    parkingStep = AutoStep.BACK_FIRST;

                    runtime.reset();
                }
                break;

            case BACK_FIRST:
                targetPose.set(0, 6.65, 0);

                if(targetPose.findDistance(poseVector) < tolerance){
                    parkingStep = AutoStep.SHOOT_FIRST;

                    runtime.reset();
                }
                break;

            case SHOOT_FIRST:
                gate.setClawState(SingleServoClaw.ClawState.OPEN);

                if (runtime.seconds() > 1) {
                    gate.setClawState(SingleServoClaw.ClawState.CLOSED);

                    parkingStep = AutoStep.SECOND_LINE;
                    runtime.reset();
                }
                break;

            case SECOND_LINE:
                targetPose.set(-30, 0, 0);

                if(targetPose.findDistance(poseVector) < tolerance){
                    parkingStep = AutoStep.BACK_SECOND;

                    runtime.reset();
                }
                break;

            case BACK_SECOND:
                targetPose.set(0, 6.65, 0);

                if(targetPose.findDistance(poseVector) < tolerance){
                    parkingStep = AutoStep.SHOOT_SECOND;

                    runtime.reset();
                }
                break;

            case SHOOT_SECOND:
                gate.openClaw();

                if (runtime.seconds() > 1) {
                    gate.closeClaw();
                    intake.setMotorIntakeState(MotorIntakeStates.RESTING);

                    parkingStep = AutoStep.PARK;
                    runtime.reset();
                }
                break;

            case PARK:
                targetPose.set(0, 20, 0);
                break;

        }


    }



    public void Telemetry(){
        //TODO: show the tolerance stuff and PID values and states
        telemetry.addData("Auton State", parkingStep);

        telemetry.addData("Pose Estimate X:", chassis.getPose()[0]);
        telemetry.addData("Pose Estimate Y:", chassis.getPose()[1]);
        telemetry.addData("Pose Estimate Angle:", chassis.getPose()[2]);

        telemetry.addData("HeadingPID error", chassis.HeadingPID.error);
        telemetry.addData("TranslationalX error", chassis.TranslationalPID_X.error);
        telemetry.addData("TranslationalY error", chassis.TranslationalPID_Y.error);
        telemetry.addData("Distance to Pose", targetPose.findDistance(chassis.getPoseVector()));

        telemetry.addData("X PID Proportion", chassis.TranslationalPID_X.P);
        telemetry.addData("Y PID Proportion", chassis.TranslationalPID_Y.P);
        telemetry.addData("Heading PID Proportion", chassis.HeadingPID.P);

        telemetry.addData("TranslationalX PID Derivative", chassis.TranslationalPID_X.D);
        telemetry.addData("TranslationalY PID Derivative", chassis.TranslationalPID_Y.D);

        telemetry.addData("X PID Integral", chassis.TranslationalPID_X.I);
        telemetry.addData("Y PID Integral", chassis.TranslationalPID_Y.I);
        telemetry.addData("Heading PID Integral", chassis.HeadingPID.I);


        telemetry.addData("runtime", runtime.seconds());

    }
}
