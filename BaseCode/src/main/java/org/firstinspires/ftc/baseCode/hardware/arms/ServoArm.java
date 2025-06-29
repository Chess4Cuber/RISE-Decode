package org.firstinspires.ftc.baseCode.hardware.arms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

public abstract class ServoArm {
    HardwareMap hardwareMap;

//    Servo[] armServos;

    List<Servo[]> armServos = new ArrayList<>();

    double servoRange = 0;

    public enum ServoArmType{
        SINGLE_SERVO,
        DOUBLE_SERVO
    }

    public enum AngleUnit {
        DEGREES,
        RADIANS
    }

    ServoArmType servoArmType;

    public ServoArm(String[] names, HardwareMap hardwareMap){
        addDegreeOfFreedom(names);
    }

    public ServoArm(HardwareMap hardwareMap){
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
        armServos.add(newServos);
    }


    //if length > 2 double servo, else single servo

    public void setPosition(double position){
        for (Servo servo:armServos.get(0)) {
            servo.setPosition(position);
        }
    }

    public void setPosition(double position, int DoF){
        for (Servo servo:armServos.get(DoF)) {
            servo.setPosition(position);
        }
    }

//    public void setAngle(double angle){
//        armServo1.setPosition(angle/servoRange);
//        armServo2.setPosition(angle/servoRange);
//    }
//
//    public void setAngle(double angle, AngleUnit unit){
//        if (unit == AngleUnit.DEGREES) {
//            armServo1.setPosition(angle/300);
//            armServo2.setPosition(angle/300);
//        }
//
//        if (unit == AngleUnit.RADIANS) {
//            armServo1.setPosition(angle/((5*Math.PI)/3));
//            armServo2.setPosition(angle/((5*Math.PI)/3));
//        }
//    }
}
