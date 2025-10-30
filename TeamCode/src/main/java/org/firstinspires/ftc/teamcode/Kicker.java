package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class Kicker {
    private final Gamepad gamepad;
    final ElapsedTime runtime;
    private boolean wasPressed;
    final long motorTime = 500;
    private long motorStartTime = -motorTime;

    DcMotor kickerMotor;
    public Kicker(HardwareMap hardwareMap, Gamepad gamepad, ElapsedTime runTime) {
        this.runtime = runTime;
        this.gamepad = gamepad;
        kickerMotor = hardwareMap.get(DcMotor.class, "kicker_motor");
        //Next three lines are for set distance mode (type = true)
        kickerMotor.setTargetPosition(0);
        kickerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        kickerMotor.setPower(0.6);
    }
    //type: true = run 1/2 rotation, false, run for 0.5 seconds
    public void tick(boolean type) {
        if (type) {
            if (gamepad.b && !this.wasPressed) {
                //28 ticks/revolution
                //kickerMotor.setTargetPosition(kickerMotor.getTargetPosition() + 14);
                this.wasPressed = true;
            } else {
                this.wasPressed = false;
            }
        }
        else {
            if (gamepad.b) {
                this.motorStartTime = runtime.now(TimeUnit.MILLISECONDS);
                //The 5 is for gearing
                kickerMotor.setTargetPosition(140 * 5);
            }
            if (runtime.now(TimeUnit.MILLISECONDS) - this.motorStartTime > motorTime) {
                kickerMotor.setTargetPosition(0);
            }
        }
    }
}
