package org.firstinspires.ftc.baseCode.hardware.extension;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.baseCode.hardware.arms.ServoArm;

import java.util.ArrayList;
import java.util.List;

public abstract class ServoLinkage {
    HardwareMap hardwareMap;
//    Servo[] armServos;
    List<Servo[]> linkageServos = new ArrayList<>();
    double servoRange = 0;
    public enum ServoLinkageType{
        SINGLE_SERVO,
        DOUBLE_SERVO
    }
    public enum AngleUnit {
        DEGREES,
        RADIANS
    }
//    public enum LinkageState{
//        EXTENDED,
//        RETRACTED
//    }
//    public LinkageState LinkageState;
    ServoLinkage.ServoLinkageType servoLinkageType;

    public ServoLinkage(String[] names, HardwareMap hardwareMap){
        addDegreeOfFreedom(names);
    }

    public ServoLinkage(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public void addDegreeOfFreedom(String[] names){
        Servo[] newServos = new Servo[names.length];
        int flipper = 1;
        for (int i = 0; i < names.length; i++){
            newServos[i] = hardwareMap.get(Servo.class, names[i]);
            if (flipper > 0) newServos[i].setDirection(Servo.Direction.FORWARD);
            else newServos[i].setDirection(Servo.Direction.REVERSE);
            flipper *= -1;
        }
        linkageServos.add(newServos);
    }


    //if length > 2 double servo, else single servo

    public void setPosition(double position){
        for (Servo servo:linkageServos.get(0)) {
            servo.setPosition(position);
        }
    }

    public void setPosition(double position, int DoF){
        for (Servo servo:linkageServos.get(DoF)) {
            servo.setPosition(position);
        }
    }


}
