package org.firstinspires.ftc.teamcode.mechanisms.RadahnSpindexerSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.hardware.extension.ServoLinkage;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnSpindexer extends ServoLinkage {

    public RadahnSpindexer(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super(hardwareMap);

        addDegreeOfFreedom(new String[]{"servoSpindexer"});


    }

}
