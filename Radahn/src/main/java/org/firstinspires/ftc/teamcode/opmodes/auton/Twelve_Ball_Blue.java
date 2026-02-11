package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.baseCode.math.Vector3D;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnChassis;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnColorSensor;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnTransfer.IntakeStates;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnTransfer.RadahnGate;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnTransfer.RadahnServoIntakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem.RadahnHoodedOuttake;
import org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem.MotorIntakeStates;
import org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem.RadahnMotorIntakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.simpleMotorOuttakeSystem.MotorOuttakeStates;
import org.firstinspires.ftc.teamcode.mechanisms.simpleMotorOuttakeSystem.RadahnMotorOuttakeSystem;

@Autonomous
public class Twelve_Ball_Blue extends LinearOpMode {

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


//TODO: FIX POSITIONS

    RadahnChassis chassis;
    RadahnMotorIntakeSystem intake;
    RadahnMotorOuttakeSystem simpleOuttake;
    RadahnGate gate;
    RadahnServoIntakeSystem transfer;
    RadahnHoodedOuttake hoodedServo;

    RadahnColorSensor colorSensors;

    AutoStep parkingStep;

    Vector3D poseVector = new Vector3D(0,0,0);
    Vector3D targetPose = new Vector3D(0, 0, 0);

    double tolerance = 1.5;

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);
        intake = new RadahnMotorIntakeSystem(gamepad1, telemetry, hardwareMap);
        simpleOuttake = new RadahnMotorOuttakeSystem(gamepad1, telemetry, hardwareMap);
        gate = new RadahnGate(gamepad1, hardwareMap);
        transfer = new RadahnServoIntakeSystem(gamepad1, hardwareMap);
        hoodedServo = new RadahnHoodedOuttake(gamepad1, telemetry, hardwareMap);

        colorSensors = new RadahnColorSensor(hardwareMap);

        parkingStep = AutoStep.AWAY_FROM_GOAL;

        while (opModeInInit()){
            intake.setMotorIntakeState(MotorIntakeStates.RESTING);
            simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.RESTING);
            hoodedServo.setHoodPosition(.3);

            telemetry.update();
        }

        while (opModeIsActive()){
            chassis.updatePose();
            intake.setPositions();
            simpleOuttake.setPositions();
            gate.setPosition();
            transfer.setPositions();

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
                simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.INTAKING);

                targetPose.set(-38, -53, 63);

                if (targetPose.findDistance(poseVector) < tolerance ){

                    parkingStep = AutoStep.SHOOTPRE;
                    runtime.reset();
                }
                break;

            case SHOOTPRE:
                gate.openClaw();
                if (runtime.seconds()>.5){
                    //simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.RESTING);
                    intake.setMotorIntakeState(MotorIntakeStates.RESTING);

                    parkingStep = AutoStep.RESET_ODO;
                    runtime.reset();
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

                targetPose.set(-40, 0, 0);

                if (targetPose.findDistance(poseVector) < tolerance ){

                    parkingStep = AutoStep.BACK_FIRST;
                    runtime.reset();
                }

                break;

            case BACK_FIRST:
                targetPose.set(0, 0, 0);
                intake.setMotorIntakeState(MotorIntakeStates.RESTING);

                if (targetPose.findDistance(poseVector) < tolerance ){
                    simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.INTAKING);
                    //intake.setMotorIntakeState(MotorIntakeStates.RESTING);

                    parkingStep = AutoStep.SHOOT_FIRST;
                    runtime.reset();
                }

                break;

            case SHOOT_FIRST:
                intake.setMotorIntakeState(MotorIntakeStates.INTAKING);


                // advance only after pusher finished
                if (runtime.seconds()>.5){
                    if (runtime.seconds() > .0) { // allow the DONE condition to be processed immediately
                        //simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.RESTING);
                        //intake.setMotorIntakeState(MotorIntakeStates.RESTING);

                        parkingStep = AutoStep.SECOND_LINE;
                        runtime.reset();
                    }
                }

                break;

            case SECOND_LINE:
                targetPose.set(0, -36, 0);

                if (targetPose.findDistance(poseVector) < tolerance ){
                    intake.setMotorIntakeState(MotorIntakeStates.INTAKING);
                    parkingStep = AutoStep.SECOND_LINE2;
                    runtime.reset();
                }
                break;

            case SECOND_LINE2:
                targetPose.set(-44, -36, 0);
                intake.setMotorIntakeState(MotorIntakeStates.INTAKING);

                if (targetPose.findDistance(poseVector) < tolerance ){
                    parkingStep = AutoStep.BACK_SECOND;
                    runtime.reset();
                }
                break;

            case BACK_SECOND:
                targetPose.set(0, 0, 0);
                intake.setMotorIntakeState(MotorIntakeStates.RESTING);

                if (targetPose.findDistance(poseVector) < tolerance ){
                    intake.setMotorIntakeState(MotorIntakeStates.INTAKING);
                    simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.INTAKING);

                    parkingStep = AutoStep.SHOOT_SECOND;
                    runtime.reset();
                }
                break;

            case SHOOT_SECOND:
                intake.setMotorIntakeState(MotorIntakeStates.INTAKING);


                // advance only after pusher finished
                if (runtime.seconds()>.5){
                    if (runtime.seconds() > .0) { // allow the DONE condition to be processed immediately
                        //simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.RESTING);
                        //intake.setMotorIntakeState(MotorIntakeStates.RESTING);

                        parkingStep = AutoStep.THIRD_LINE;
                        runtime.reset();
                    }
                }

                break;

            case THIRD_LINE:
                targetPose.set(0, -70, 0);
                intake.setMotorIntakeState(MotorIntakeStates.INTAKING);

                if (targetPose.findDistance(poseVector) < tolerance ){
                    parkingStep = AutoStep.THIRD_LINE2;
                    runtime.reset();
                }
                break;

            case THIRD_LINE2:
                targetPose.set(-47, -70, 0);
                if (targetPose.findDistance(poseVector) < tolerance ){
                    parkingStep = AutoStep.BACK_THIRD;
                    runtime.reset();
                }
                break;

            case BACK_THIRD:
                targetPose.set(0, 0, 0);
                intake.setMotorIntakeState(MotorIntakeStates.RESTING);


                if (targetPose.findDistance(poseVector) < tolerance ){
                    intake.setMotorIntakeState(MotorIntakeStates.INTAKING);
                    simpleOuttake.setMotorOuttakeState(MotorOuttakeStates.INTAKING);

                    parkingStep = AutoStep.SHOOT_THIRD;
                    runtime.reset();
                }
                break;


            case SHOOT_THIRD:
                intake.setMotorIntakeState(MotorIntakeStates.INTAKING);


                // advance only after pusher finished
                if (runtime.seconds()>.5){
                    if (runtime.seconds() > .0) { // allow the DONE condition to be processed immediately
                        //intake.setMotorIntakeState(MotorIntakeStates.RESTING);

                        parkingStep = AutoStep.PARK;
                        runtime.reset();
                    }
                }

            case PARK:
                targetPose.set(-35, 0, 0);
                break;

        }
    }


    public void transferCommence(){
        gate.openClaw();
        transfer.setServoIntakeState(IntakeStates.INTAKING);
    }

    public void transferStop(){
        gate.closeClaw();
        transfer.setServoIntakeState(IntakeStates.RESTING);
    }

    //ADD WHEN NEAR GATE
    public boolean colorCheck(){
        boolean check = false;

        if(colorSensors.seesGreenOrPurple(0)){
            check = true;

        } else if (colorSensors.seesGreenOrPurple(1)) {
            check = true;
        }

        return check;
    }

    public boolean colorTransition(){
        int count = 0;

        boolean check = false;

        while(runtime.seconds() < 4){

            if(colorCheck()){
                count++;
            }

        }

        if(count == 3){
            check = true;
        }

        return check;

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
