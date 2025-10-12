package org.firstinspires.ftc.teamcode.mechanisms.RadahnPlanche;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnSlidesSystem {
    Gamepad gamepad1;
    Telemetry telemetry;
    RadahnnSlides slides;

    boolean lastToggleX = false;
    boolean lastToggleB = false;
    public SlidesStates slidesState;

    public RadahnSlidesSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        slides = new RadahnnSlides(gamepad1, telemetry, hardwareMap);

        slidesState = SlidesStates.GROUND;
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void setPositions(){
        switch (slidesState){
            case GROUND:
                slides.setExtension(0);
                break;
            case EXTENDED:
                slides.setExtension(30);
                break;

        }
    }
    public void controllerInput(){
        switch (slidesState) {
            case GROUND:
                if ((gamepad1.x != lastToggleX) && gamepad1.x) {
                    slidesState = SlidesStates.EXTENDED;
                }
                break;

            case EXTENDED:
                if((gamepad1.b != lastToggleB) && gamepad1.b){
                    slidesState = SlidesStates.GROUND;
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
