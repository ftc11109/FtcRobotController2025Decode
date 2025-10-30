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
        kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        kickerMotor.setPower(0);
    }

    public void tick(double speed) {
        if (gamepad.b) {
            this.motorStartTime = runtime.now(TimeUnit.MILLISECONDS);
            kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            kickerMotor.setPower(speed);
        }
        if (runtime.now(TimeUnit.MILLISECONDS) - this.motorStartTime > motorTime || kickerMotor.getCurrentPosition() > 14) {
            kickerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            kickerMotor.setTargetPosition(0);
        }
    }
}
