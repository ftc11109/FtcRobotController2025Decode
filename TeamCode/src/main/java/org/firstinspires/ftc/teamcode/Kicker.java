package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class Kicker {
    private final Gamepad gamepad;
    private ElapsedTime runtime;
    private boolean wasPressed;
    private long motorTime = 500;
    private long motorStartTime = -motorTime;

    DcMotor kickerMotor;
    public Kicker(HardwareMap hardwareMap, Gamepad gamepad, ElapsedTime runTime) {
        this.runtime = runTime;
        this.gamepad = gamepad;
        kickerMotor = hardwareMap.get(DcMotor.class, "kicker_motor");
        //Next three lines are for set distance mode (type = true)
        //kickerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //kickerMotor.setTargetPosition(0);
        //kickerMotor.setPower(0.2);
    }
    //type: true = run 1/2 rotation, false, run for 0.5 seconds
    public int tick(boolean type) {
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
            if(gamepad.b) {
                this.motorStartTime = runtime.now(TimeUnit.MILLISECONDS);
                kickerMotor.setPower(0.7);
            }
            if(runtime.now(TimeUnit.MILLISECONDS) - this.motorStartTime > motorTime) {
                kickerMotor.setPower(0);
            }
        }
        if(gamepad.right_bumper) {
            gamepad.rumble(500);
        }
        return kickerMotor.getCurrentPosition();
    }
}
