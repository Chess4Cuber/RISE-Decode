package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.baseCode.hardware.claws.SingleServoClaw;
import org.firstinspires.ftc.baseCode.math.Vector3D;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnChassis;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnPusher;
import org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem.RadahnHoodedOuttake;
import org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem.MotorIntakeStates;
import org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem.RadahnMotorIntakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.simpleMotorOuttakeSystem.MotorOuttakeStates;
import org.firstinspires.ftc.teamcode.mechanisms.simpleMotorOuttakeSystem.RadahnMotorOuttakeSystem;

@Autonomous
public class Six_Ball_Far_Blue extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    public enum AutoStep{
        AWAY_FROM_GOAL,
        SHOOTPRE,
        RESET_ODO,
        FIRST_LINE,
        BACK_FIRST,
        SHOOT_FIRST,
        SECOND_LINE,
        SECOND_LINE2,
        BACK_SECOND,
        SHOOT_SECOND,
        THIRD_LINE,
        THIRD_LINE2,
        BACK_THIRD,
        SHOOT_THIRD,
        PARK
    }

    public enum PusherState {
        REVUP,
        SHOOT_FIRST,
        SHOOT_SECOND,
        SHOOT_THIRD,
        SHOOT_FOURTH,
        SHOOT_FIFTH,
        SHOOT_SIXTH,
        DONE
    }

    PusherState pusherState;
    RadahnChassis chassis;
    RadahnMotorIntakeSystem intake;
    RadahnMotorOuttakeSystem simpleOuttake;
    RadahnPusher pusher;
    RadahnHoodedOuttake hoodedServo;

    AutoStep parkingStep;

    Vector3D poseVector = new Vector3D(0,0,0);
    Vector3D targetPose = new Vector3D(0, 0, 0);

    double tolerance = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);
        intake = new RadahnMotorIntakeSystem(gamepad1, telemetry, hardwareMap);
        simpleOuttake = new RadahnMotorOuttakeSystem(gamepad1, telemetry, hardwareMap);
        pusher = new RadahnPusher(gamepad1, hardwareMap);
        hoodedServo = new RadahnHoodedOuttake(gamepad1, telemetry, hardwareMap);

        parkingStep = AutoStep.AWAY_FROM_GOAL;
        pusherState = PusherState.REVUP;

        while (opModeInInit()){
            intake.setMotorIntakeState(MotorIntakeStates.RESTING);
            simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.RESTING);
            hoodedServo.setHoodPosition(-.5);

            telemetry.update();
        }

        while (opModeIsActive()){
            chassis.updatePose();
            intake.setPositions();
            simpleOuttake.setPositions();
            pusher.setPosition();

            autonBlue();

            chassis.goToPosePID(targetPose);
            poseVector.set(chassis.odo.getX(), chassis.odo.getY(), chassis.getPose()[2]);

            Telemetry();

            telemetry.update();
        }
    }

    public void autonBlue(){

        switch (parkingStep){
            case AWAY_FROM_GOAL:
                simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.FAR_SHOT);

                targetPose.set(-20, 0, 0);

                if (targetPose.findDistance(poseVector) < tolerance ){

                    parkingStep = AutoStep.SHOOTPRE;
                    runtime.reset();
                    pusherState = PusherState.REVUP;
                }
                break;

            case SHOOTPRE:
                switch (pusherState) {
                    case REVUP:
                        if (runtime.seconds() > 4) {
                            pusher.setClawState(SingleServoClaw.ClawState.OPEN);
                            pusherState = PusherState.SHOOT_FIRST;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_FIRST:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.CLOSED);
                            pusherState = PusherState.SHOOT_SECOND;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_SECOND:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.RESET);
                            intake.setMotorIntakeState(MotorIntakeStates.INTAKING);

                            pusherState = PusherState.SHOOT_THIRD;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_THIRD:

                        if (runtime.seconds() > 3) {
                            pusher.setClawState(SingleServoClaw.ClawState.MIDDLE);
                            pusherState = PusherState.SHOOT_FOURTH;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_FOURTH:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.CLOSED);
                            pusherState = PusherState.SHOOT_FIFTH;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_FIFTH:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.OPEN);
                            intake.setMotorIntakeState(MotorIntakeStates.INTAKING);
                            pusherState = PusherState.SHOOT_SIXTH;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_SIXTH:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.RESET);
                            pusherState = PusherState.DONE;
                            runtime.reset();
                        }
                        break;

                    case DONE:
                        if(runtime.seconds() > .03){
                            chassis.odo.setPose(0, 0, 0);
                            chassis.odo.resetEncoderDeltas();
                        }

                        break;
                }

                if (pusherState == PusherState.DONE){
                    //simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.RESTING);
                    intake.setMotorIntakeState(MotorIntakeStates.RESTING);

                    parkingStep = AutoStep.RESET_ODO;
                    runtime.reset();
                    pusherState = PusherState.REVUP;
                }
                break;

            case RESET_ODO:
                chassis.odo.setPose(0, 0, 0);
                chassis.odo.resetEncoderDeltas();

                targetPose.set(0, -23, 0);

                if(runtime.seconds()>.2){
                    parkingStep = AutoStep.FIRST_LINE;
                    runtime.reset();
                }

                break;

            case FIRST_LINE:

                intake.setMotorIntakeState(MotorIntakeStates.INTAKING);

                targetPose.set(-35, 0, 0);

                if (targetPose.findDistance(poseVector) < tolerance ){

                    parkingStep = AutoStep.BACK_FIRST;
                    runtime.reset();
                }

                break;

            case BACK_FIRST:
                targetPose.set(0, 0, 0);

                if (targetPose.findDistance(poseVector) < tolerance ){
                    simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.INTAKING);
                    intake.setMotorIntakeState(MotorIntakeStates.RESTING);

                    parkingStep = AutoStep.SHOOT_FIRST;
                    runtime.reset();
                    pusherState = PusherState.REVUP;
                }

                break;

            case SHOOT_FIRST:
                switch (pusherState) {
                    case REVUP:
                        if (runtime.seconds() > 4) {
                            pusher.setClawState(SingleServoClaw.ClawState.OPEN);
                            pusherState = PusherState.SHOOT_FIRST;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_FIRST:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.CLOSED);
                            pusherState = PusherState.SHOOT_SECOND;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_SECOND:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.RESET);
                            intake.setMotorIntakeState(MotorIntakeStates.INTAKING);

                            pusherState = PusherState.SHOOT_THIRD;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_THIRD:

                        if (runtime.seconds() > 3) {
                            pusher.setClawState(SingleServoClaw.ClawState.MIDDLE);
                            pusherState = PusherState.SHOOT_FOURTH;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_FOURTH:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.CLOSED);
                            pusherState = PusherState.SHOOT_FIFTH;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_FIFTH:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.OPEN);
                            intake.setMotorIntakeState(MotorIntakeStates.INTAKING);
                            pusherState = PusherState.SHOOT_SIXTH;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_SIXTH:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.RESET);
                            pusherState = PusherState.DONE;
                            runtime.reset();
                        }
                        break;

                    case DONE:
                        if(runtime.seconds() > .03){
                            chassis.odo.setPose(0, 0, 0);
                            chassis.odo.resetEncoderDeltas();
                        }

                        break;
                }

                // advance only after pusher finished
                if (pusherState == PusherState.DONE){
                    if (runtime.seconds() > .0) {
                        simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.RESTING);
                        intake.setMotorIntakeState(MotorIntakeStates.RESTING);

                        parkingStep = AutoStep.PARK;
                        runtime.reset();
                        pusherState = PusherState.REVUP; // reset for next use
                    }
                }

                break;

            case SECOND_LINE:
                targetPose.set(-28.5, -36, 0);

                if (targetPose.findDistance(poseVector) < tolerance ){
                    intake.setMotorIntakeState(MotorIntakeStates.INTAKING);
                    parkingStep = AutoStep.SECOND_LINE2;
                    runtime.reset();
                }
                break;

            case SECOND_LINE2:
                targetPose.set(-50, -36, 0);

                if (targetPose.findDistance(poseVector) < tolerance ){
                    parkingStep = AutoStep.BACK_SECOND;
                    runtime.reset();
                }

            case BACK_SECOND:
                targetPose.set(0, 0, 0);

                if (targetPose.findDistance(poseVector) < tolerance ){
                    intake.setMotorIntakeState(MotorIntakeStates.INTAKING);
                    simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.INTAKING);

                    parkingStep = AutoStep.SHOOT_SECOND;
                    runtime.reset();
                    pusherState = PusherState.REVUP;
                }
                break;

            case SHOOT_SECOND:
                switch (pusherState) {
                    case REVUP:
                        if (runtime.seconds() > 4) {
                            pusher.setClawState(SingleServoClaw.ClawState.OPEN);
                            pusherState = PusherState.SHOOT_FIRST;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_FIRST:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.CLOSED);
                            pusherState = PusherState.SHOOT_SECOND;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_SECOND:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.RESET);
                            intake.setMotorIntakeState(MotorIntakeStates.INTAKING);

                            pusherState = PusherState.SHOOT_THIRD;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_THIRD:

                        if (runtime.seconds() > 3) {
                            pusher.setClawState(SingleServoClaw.ClawState.MIDDLE);
                            pusherState = PusherState.SHOOT_FOURTH;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_FOURTH:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.CLOSED);
                            pusherState = PusherState.SHOOT_FIFTH;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_FIFTH:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.OPEN);
                            intake.setMotorIntakeState(MotorIntakeStates.INTAKING);
                            pusherState = PusherState.SHOOT_SIXTH;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_SIXTH:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.RESET);
                            pusherState = PusherState.DONE;
                            runtime.reset();
                        }
                        break;

                    case DONE:
                        if(runtime.seconds() > .03){
                            chassis.odo.setPose(0, 0, 0);
                            chassis.odo.resetEncoderDeltas();
                        }

                        break;
                }

                if (pusherState == PusherState.DONE){
                    simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.RESTING);
                    intake.setMotorIntakeState(MotorIntakeStates.INTAKING);

                    parkingStep = AutoStep.THIRD_LINE;
                    runtime.reset();
                    pusherState = PusherState.REVUP;
                }
                break;

            case THIRD_LINE:
                targetPose.set(-28.5, -47.5, 0);
                intake.setMotorIntakeState(MotorIntakeStates.INTAKING);

                if (targetPose.findDistance(poseVector) < tolerance ){
                    parkingStep = AutoStep.THIRD_LINE2;
                    runtime.reset();
                }
                break;

            case THIRD_LINE2:
                targetPose.set(-50, -47.5, 0);
                if (targetPose.findDistance(poseVector) < tolerance ){
                    parkingStep = AutoStep.BACK_THIRD;
                    runtime.reset();
                }
                break;

            case BACK_THIRD:
                targetPose.set(0, 0, 0);

                if (targetPose.findDistance(poseVector) < tolerance ){
                    intake.setMotorIntakeState(MotorIntakeStates.INTAKING);
                    simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.INTAKING);

                    parkingStep = AutoStep.SHOOT_THIRD;
                    runtime.reset();
                    pusherState = PusherState.REVUP;
                }
                break;

            case SHOOT_THIRD:
                switch (pusherState) {
                    case REVUP:
                        if (runtime.seconds() > 4) {
                            pusher.setClawState(SingleServoClaw.ClawState.OPEN);
                            pusherState = PusherState.SHOOT_FIRST;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_FIRST:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.CLOSED);
                            pusherState = PusherState.SHOOT_SECOND;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_SECOND:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.RESET);
                            intake.setMotorIntakeState(MotorIntakeStates.INTAKING);

                            pusherState = PusherState.SHOOT_THIRD;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_THIRD:

                        if (runtime.seconds() > 3) {
                            pusher.setClawState(SingleServoClaw.ClawState.MIDDLE);
                            pusherState = PusherState.SHOOT_FOURTH;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_FOURTH:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.CLOSED);
                            pusherState = PusherState.SHOOT_FIFTH;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_FIFTH:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.OPEN);
                            intake.setMotorIntakeState(MotorIntakeStates.INTAKING);
                            pusherState = PusherState.SHOOT_SIXTH;
                            runtime.reset();
                        }
                        break;

                    case SHOOT_SIXTH:

                        if (runtime.seconds() > 1) {
                            pusher.setClawState(SingleServoClaw.ClawState.RESET);
                            pusherState = PusherState.DONE;
                            runtime.reset();
                        }
                        break;

                    case DONE:
                        if(runtime.seconds() > .03){
                            chassis.odo.setPose(0, 0, 0);
                            chassis.odo.resetEncoderDeltas();
                        }

                        break;
                }

                if (pusherState == PusherState.DONE){
                    simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.RESTING);
                    intake.setMotorIntakeState(MotorIntakeStates.RESTING);

                    parkingStep = AutoStep.PARK;
                    runtime.reset();
                    pusherState = PusherState.REVUP;
                }
                break;

            case PARK:
                targetPose.set(-35, 0, 0);
                break;

        }
    }

    public void pusherMove1(){
        //Rev up flywheel and shoot first ball
        if(runtime.seconds() > 2){
            pusher.setClawState(SingleServoClaw.ClawState.OPEN);
            runtime.reset();
        }

        if(runtime.seconds() > .5){
            pusher.setClawState(SingleServoClaw.ClawState.CLOSED);
            runtime.reset();
        }
        //Shoot second ball
        if(runtime.seconds() > .5){
            pusher.setClawState(SingleServoClaw.ClawState.OPEN);
            runtime.reset();
        }

        if(runtime.seconds() > .5){
            pusher.setClawState(SingleServoClaw.ClawState.CLOSED);
            runtime.reset();
        }
        //Shoot third ball
        if(runtime.seconds() > .5){
            pusher.setClawState(SingleServoClaw.ClawState.OPEN);
            runtime.reset();
        }

        if(runtime.seconds() > .5){
            pusher.setClawState(SingleServoClaw.ClawState.CLOSED);
            runtime.reset();
        }
    }

    public void pusherMove() {
        switch (pusherState) {
            case REVUP:
                if (runtime.seconds() > 4) {
                    pusher.setClawState(SingleServoClaw.ClawState.OPEN);
                    pusherState = PusherState.SHOOT_FIRST;
                    runtime.reset();
                }
                break;

            case SHOOT_FIRST:

                if (runtime.seconds() > 1) {
                    pusher.setClawState(SingleServoClaw.ClawState.CLOSED);
                    pusherState = PusherState.SHOOT_SECOND;
                    runtime.reset();
                }
                break;

            case SHOOT_SECOND:

                if (runtime.seconds() > 1) {
                    pusher.setClawState(SingleServoClaw.ClawState.RESET);
                    intake.setMotorIntakeState(MotorIntakeStates.INTAKING);

                    pusherState = PusherState.SHOOT_THIRD;
                    runtime.reset();
                }
                break;

            case SHOOT_THIRD:

                if (runtime.seconds() > 3) {
                    pusher.setClawState(SingleServoClaw.ClawState.MIDDLE);
                    pusherState = PusherState.SHOOT_FOURTH;
                    runtime.reset();
                }
                break;

            case SHOOT_FOURTH:

                if (runtime.seconds() > 1) {
                    pusher.setClawState(SingleServoClaw.ClawState.CLOSED);
                    pusherState = PusherState.SHOOT_FIFTH;
                    runtime.reset();
                }
                break;

            case SHOOT_FIFTH:

                if (runtime.seconds() > 1) {
                    pusher.setClawState(SingleServoClaw.ClawState.OPEN);
                    intake.setMotorIntakeState(MotorIntakeStates.INTAKING);
                    pusherState = PusherState.SHOOT_SIXTH;
                    runtime.reset();
                }
                break;

            case SHOOT_SIXTH:

                if (runtime.seconds() > 1) {
                    pusher.setClawState(SingleServoClaw.ClawState.RESET);
                    pusherState = PusherState.DONE;
                    runtime.reset();
                }
                break;

            case DONE:
                if(runtime.seconds() > .03){
                    chassis.odo.setPose(0, 0, 0);
                    chassis.odo.resetEncoderDeltas();
                }

                break;
        }
    }

    public void Telemetry(){
        //TODO: show the tolerance stuff and PID values and states
        telemetry.addData("Auton State", parkingStep);

        telemetry.addData("Motor RPM:", simpleOuttake.getRPM());

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
