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
public class Nine_Ball_BLUE extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();


    public enum AutoStep{
        AWAY_FROM_GOAL,
        SHOOTPRE,
        FIRST_LINE,
        BACK_FIRST,
        SHOOT_FIRST,
        SECOND_LINE,
        BACK_SECOND,
        SHOOT_SECOND,
        THIRD_LINE,
        BACK_THIRD,
        SHOOT_THIRD,

    }

    RadahnChassis chassis;
    RadahnMotorIntakeSystem intake;
    RadahnMotorOuttakeSystem simpleOuttake;
    RadahnPusher pusher;
    RadahnHoodedOuttake hoodedServo;

    AutoStep parkingStep;

    Vector3D poseVector = new Vector3D(0,0,0);
    Vector3D targetPose = new Vector3D(0, 0, 0);

    double tolerance = 1.5;

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);
        intake = new RadahnMotorIntakeSystem(gamepad1, telemetry, hardwareMap);
        simpleOuttake = new RadahnMotorOuttakeSystem(gamepad1, telemetry, hardwareMap);
        pusher = new RadahnPusher(gamepad1, hardwareMap);
        hoodedServo = new RadahnHoodedOuttake(gamepad1, telemetry, hardwareMap);

        parkingStep = AutoStep.AWAY_FROM_GOAL;

        while (opModeInInit()){
            intake.setMotorIntakeState(MotorIntakeStates.RESTING);
            simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.RESTING);
            hoodedServo.setHoodPosition(-.47);

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
                targetPose.set(-35, -70, 63);

                if (targetPose.findDistance(poseVector) < tolerance ){
                    simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.INTAKING);
                    parkingStep = AutoStep.SHOOTPRE;
                    runtime.reset();
                }
                break;

            case SHOOTPRE:
                pusherMove();

                if (runtime.seconds() > .3 ){
                    simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.RESTING);
                    parkingStep = AutoStep.FIRST_LINE;
                    runtime.reset();
                }
                break;

            case FIRST_LINE:
                intake.setMotorIntakeState(MotorIntakeStates.INTAKING);

                targetPose.set(-60, -20, 63);

                if (targetPose.findDistance(poseVector) < tolerance ){
                    intake.setMotorIntakeState(MotorIntakeStates.RESTING);
                    parkingStep = AutoStep.BACK_FIRST;
                    runtime.reset();
                }

                break;

            case BACK_FIRST:
                targetPose.set(-35, -70, 63);

                if (targetPose.findDistance(poseVector) < tolerance ){
                    intake.setMotorIntakeState(MotorIntakeStates.RESTING);
                    simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.INTAKING);

                    parkingStep = AutoStep.SHOOT_FIRST;
                    runtime.reset();
                }

                break;

            case SHOOT_FIRST:
                pusherMove();
                intake.setMotorIntakeState(MotorIntakeStates.INTAKING);

                if (runtime.seconds() > .3 ){
                    simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.RESTING);
                    parkingStep = AutoStep.SECOND_LINE;
                    runtime.reset();
                }

                break;

            case SECOND_LINE:
                break;
        }
    }

    public void pusherMove(){
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
