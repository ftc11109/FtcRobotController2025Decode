package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Kicker {
    private final Gamepad gamepad;
    private boolean wasPressed;
    DcMotor kickerMotor;
    public Kicker(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        kickerMotor = hardwareMap.get(DcMotor.class, "kicker_motor");
        kickerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        kickerMotor.setTargetPosition(0);
        kickerMotor.setPower(0.2);
    }

    public int tick() {
        if(gamepad.b && !this.wasPressed) {
            //28 ticks/revolution
            kickerMotor.setTargetPosition(kickerMotor.getTargetPosition() + 14);
            this.wasPressed = true;
        }
        else {
            this.wasPressed = false;
        }
        return kickerMotor.getCurrentPosition();
    }
}
