package org.firstinspires.ftc.teamcode.mechanisms.otherMechanisms.linkageArmSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.hardware.extension.ServoLinkage;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnLinkage extends ServoLinkage {

    public RadahnLinkage(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super(hardwareMap);

        addDegreeOfFreedom(new String[]{"leftServoLinkage", "rightServoLinkage"});
        addDegreeOfFreedom(new String[]{"intakeRotator"});


    }

}
