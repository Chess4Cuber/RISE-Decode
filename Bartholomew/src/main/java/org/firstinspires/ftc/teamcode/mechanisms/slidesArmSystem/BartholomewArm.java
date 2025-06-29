package org.firstinspires.ftc.teamcode.mechanisms.slidesArmSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.hardware.arms.ServoArm;

public class BartholomewArm extends ServoArm {
    public BartholomewArm(HardwareMap hardwareMap){
        super(hardwareMap);

        addDegreeOfFreedom(new String[]{"armServoLeft", "armServoRight"});
        addDegreeOfFreedom(new String[]{"wrist"});
    }
}
