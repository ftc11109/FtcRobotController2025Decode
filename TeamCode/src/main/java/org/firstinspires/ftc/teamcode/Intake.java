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

    public void tick(double speed) {
        if(gamepad.dpad_left) {
            //stop
            intakeMotor.setPower(0);
        }
        else if(gamepad.dpad_down) {
            //counterclockwise
            intakeMotor.setPower(speed);
        }
        else if(gamepad.dpad_up) {
            //clockwise
            intakeMotor.setPower(-speed);
        }
    }
}
