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
import org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem.TurretHoodStates;
import org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem.MotorIntakeStates;
import org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem.RadahnMotorIntakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.turretSystem.RadahnTurretSystem;

@Autonomous
public class Twelve_Ball_Blue extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    public enum AutoStep{
        MOVE_TURN_TURRET,
        REVUP,
        SHOOTPRE,
        FIRST_LINE,
        FIRST_LINE2,
        BACK_FIRST1,
        BACK_FIRST2,
        SHOOT_FIRST,
        OPEN_CLASSIFIER1,
        OPEN_CLASSIFIER2,
        INTAKE_CLASSIFIER,
        BACK_CLASSIFIER1,
        BACK_CLASSIFIER2,
        SHOOT_CLASSIFIER,
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

    double tolerance = 3;
    double shootTime = 2;
    double count = 0;

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
            gate.setClawState(SingleServoClaw.ClawState.OPEN);

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
                targetPose.set(20.06, -33.54, 0);
                hoodedOuttakeSystem.setMotorOuttakeState(TurretHoodStates.OUTTAKING);
                intake.setMotorIntakeState(MotorIntakeStates.INTAKING);

                if (targetPose.findDistance(poseVector) < tolerance ){
                    parkingStep = AutoStep.REVUP;
                    runtime.reset();
                }
                break;

            case REVUP:
                if(runtime.seconds() > 2.5){
                    parkingStep = AutoStep.SHOOTPRE;
                    runtime.reset();
                }
                break;

            case SHOOTPRE:
                gate.setClawState(SingleServoClaw.ClawState.CLOSED);

                if (runtime.seconds() > shootTime) {
                    gate.setClawState(SingleServoClaw.ClawState.OPEN);

                    parkingStep = AutoStep.FIRST_LINE;
                    runtime.reset();
                }
                break;

            case FIRST_LINE:
                targetPose.set(10.63, -60.45, 0);

                if(targetPose.findDistance(poseVector) < tolerance){

                    parkingStep = AutoStep.FIRST_LINE2;
                    runtime.reset();
                }
                break;

            case FIRST_LINE2:
                targetPose.set(-14.26, -60.45, 0);

                if(targetPose.findDistance(poseVector) < tolerance){
                    parkingStep = AutoStep.BACK_FIRST1;

                    runtime.reset();
                }
                break;

            case BACK_FIRST1:
                targetPose.set(10.63, -60.45, 0);

                if(targetPose.findDistance(poseVector) < tolerance){
                    parkingStep = AutoStep.BACK_FIRST2;

                    runtime.reset();
                }
                break;

            case BACK_FIRST2:
                targetPose.set(20.06, -33.54, 0);

                if(targetPose.findDistance(poseVector) < tolerance){
                    parkingStep = AutoStep.SHOOT_FIRST;

                    runtime.reset();
                }
                break;

            case SHOOT_FIRST:
                gate.setClawState(SingleServoClaw.ClawState.CLOSED);

                if (runtime.seconds() > shootTime) {
                    gate.setClawState(SingleServoClaw.ClawState.OPEN);
                    intake.setMotorIntakeState(MotorIntakeStates.RESTING);

                    parkingStep = AutoStep.OPEN_CLASSIFIER1;
                    runtime.reset();
                }
                break;
            case OPEN_CLASSIFIER1:
                targetPose.set(5.60, -54.65,0);

                if(targetPose.findDistance(poseVector) < tolerance){
                    intake.setMotorIntakeState(MotorIntakeStates.INTAKING);

                    parkingStep = AutoStep.OPEN_CLASSIFIER2;
                    runtime.reset();
                }
                break;

            case OPEN_CLASSIFIER2:
                targetPose.set(-11.89,-54.65,0);

                if(targetPose.findDistance(poseVector) < tolerance){
                    intake.setMotorIntakeState(MotorIntakeStates.INTAKING);

                    parkingStep = AutoStep.INTAKE_CLASSIFIER;
                    runtime.reset();
                }
                break;

            case INTAKE_CLASSIFIER:
                if(runtime.seconds() > 3){
                    parkingStep = AutoStep.BACK_CLASSIFIER1;
                    runtime.reset();
                }
                break;

            case BACK_CLASSIFIER1:
                targetPose.set(5.60, -54.65,0);

                if(targetPose.findDistance(poseVector) < tolerance){
                    parkingStep = AutoStep.BACK_CLASSIFIER2;
                    runtime.reset();
                }
                break;

            case BACK_CLASSIFIER2:
                targetPose.set(20.06, -33.54, 0);
                count++;

                if(targetPose.findDistance(poseVector) < tolerance){
                    parkingStep = AutoStep.SHOOT_CLASSIFIER;
                    runtime.reset();
                }
                break;

            case SHOOT_CLASSIFIER:
                gate.setClawState(SingleServoClaw.ClawState.OPEN);

                if (count == 3 && (runtime.seconds() > shootTime)) {
                    gate.setClawState(SingleServoClaw.ClawState.OPEN);
                    intake.setMotorIntakeState(MotorIntakeStates.RESTING);

                    parkingStep = AutoStep.BACK_SECOND;
                    runtime.reset();
                }else {
                    parkingStep = AutoStep.OPEN_CLASSIFIER1;
                    runtime.reset();
                }

            case SECOND_LINE:
                targetPose.set(-11, -60, 0);

                if(targetPose.findDistance(poseVector) < tolerance){
                    parkingStep = AutoStep.BACK_SECOND;

                    runtime.reset();
                }
                break;

            case BACK_SECOND:
                targetPose.set(-15, 0, 0);

                if(targetPose.findDistance(poseVector) < tolerance){
                    parkingStep = AutoStep.SHOOT_SECOND;

                    runtime.reset();
                }
                break;

            case SHOOT_SECOND:
                gate.openClaw();

                if (runtime.seconds() > .5) {
                    gate.closeClaw();
                    intake.setMotorIntakeState(MotorIntakeStates.RESTING);

                    parkingStep = AutoStep.PARK;
                    runtime.reset();
                }
                break;

            case PARK:
                targetPose.set(0, 0, 0);
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
