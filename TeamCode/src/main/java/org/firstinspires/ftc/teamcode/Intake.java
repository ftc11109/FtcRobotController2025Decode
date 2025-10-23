package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final Gamepad gamepad;
    DcMotor intakeMotor;

    public Intake(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
    }

    public void tick() {
        if(gamepad.dpad_left) {
            intakeMotor.setPower(0);
        }
        else if(gamepad.dpad_down) {
            intakeMotor.setPower(0.05);
        }
        else if(gamepad.dpad_up) {
            intakeMotor.setPower(-0.05);
        }
    }
}
