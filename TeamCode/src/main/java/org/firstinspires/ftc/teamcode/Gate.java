package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Gate {
    Servo gateServo;
    public Gate(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, "gate_servo");
    }
    public void gateUp() {
        gateServo.setPosition(1);
    }
    public void gateDown() {
        gateServo.setPosition(0);
    }
}
