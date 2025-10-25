package org.firstinspires.ftc.teamcode.mechanisms.otherMechanisms.slidesArmSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OptimusSlidesArmSystem {
    Gamepad gamepad1;
    Telemetry telemetry;
    OptimusSlides slides;
//    OptimusArm arm;

    boolean lastToggleX = false;
    boolean lastToggleB = false;
    public OldSlidesStates slidesState;

    public OptimusSlidesArmSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        slides = new OptimusSlides(gamepad1, telemetry, hardwareMap);
        //arm = new OptimusArm(hardwareMap);

        slidesState = OldSlidesStates.GROUND;
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void setPositions(){
        switch (slidesState){
            case GROUND:
                slides.setExtension(0);
                //arm.setPosition(0.5,0); // first arm stage
                //arm.setPosition(0.65,1); //rotator
                //arm.setPosition(.97,2); //wrist
                break;
            case TRANSFER:
                slides.setExtension(0);
                //arm.setPosition(0.64,0); // first arm stage
                //arm.setPosition(0.5,1); //rotator
                //arm.setPosition(.97,2); //wrist
                break;
            case HIGH_JUNCTION:
                slides.setExtension(50);
//                arm.setPosition(0.2,0); // first arm stage
//                arm.setPosition(0.1,1); //rotator
//                arm.setPosition(0,2); //wrist
                break;
        }
    }
    public void controllerInput(){
        switch (slidesState) {
            case GROUND:
                if ((gamepad1.x != lastToggleX) && gamepad1.x) {
                    slidesState = OldSlidesStates.HIGH_JUNCTION;
                }
                if((gamepad1.b != lastToggleB) && gamepad1.b){
                    slidesState = OldSlidesStates.TRANSFER;
                }
                break;
            case TRANSFER:
                if((gamepad1.b != lastToggleB) && gamepad1.b){
                    slidesState = OldSlidesStates.HIGH_JUNCTION;
                }
                break;
            case HIGH_JUNCTION:
                if ((gamepad1.x != lastToggleX) && gamepad1.x) {
                    slidesState = OldSlidesStates.GROUND;
                }
                break;
        }
        lastToggleX = gamepad1.x;
        lastToggleB = gamepad1.b;
    }

    public void setTelemetry(){
        telemetry.addData("Get Extension", slides.getExtension());
        telemetry.addData("SlidesLeft_Encoder", slides.motors[0].getCurrPosTicks());
        telemetry.addData("SlidesRight_Encoder", slides.motors[1].getCurrPosTicks());
        telemetry.addData("Proportional Term", slides.slidesPID.P);
        telemetry.addData("Derivative Term", slides.slidesPID.D);
        telemetry.addData("Integral Term", slides.slidesPID.I);
    }

}
