package org.firstinspires.ftc.teamcode.mechanisms.otherMechanisms.linkageArmSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnLinkageArmSystem {

    Gamepad gamepad1;
    RadahnLinkage linkage;
    public LinkageStates linkageState;
    boolean lastToggleA = false;
    public RadahnLinkageArmSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        linkage = new RadahnLinkage(gamepad1, telemetry, hardwareMap);

        this.gamepad1 = gamepad1;
        linkageState = LinkageStates.RETRACTED;

    }

    public void setPositions(){
        switch (linkageState) {
            case RETRACTED:
                linkage.setPosition(.47, 0);
                break;
            case EXTENDED:
                linkage.setPosition(.7, 0);
                break;
        }
    }

    public void controllerInput(){
        switch (linkageState) {
            case RETRACTED:
                if ((gamepad1.a != lastToggleA) && gamepad1.a) {
                    linkageState = LinkageStates.EXTENDED;
                }

                break;
            case EXTENDED:
                if ((gamepad1.a != lastToggleA) && gamepad1.a) {
                    linkageState = LinkageStates.RETRACTED;
                }
                break;
        }
        lastToggleA = gamepad1.a;
    }

}
