package org.firstinspires.ftc.teamcode.mechanisms.RadahnSpindexerSystem.ArmSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.hardware.arms.ServoArm;

public class RadahnArm extends ServoArm {

    public RadahnArm(HardwareMap hardwareMap){
        super(hardwareMap);

        addDegreeOfFreedom(new String[]{"pusher"});
    }

}
