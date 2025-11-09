package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Gate {
    public Gate(HardwareMap hardwareMap) {
        Servo gateServo;
        gateServo = hardwareMap.get(Servo.class, "gate_servo");
    }
    public void gateUp() {

    }
    public void gateDown() {

    }
}
