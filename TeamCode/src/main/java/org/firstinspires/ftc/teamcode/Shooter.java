package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private final Gamepad gamepad;
    DcMotorEx shooterMotor;

    public Shooter(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void tick(int rpm) {
        if(gamepad.a) {
            shooterMotor.setVelocity(28 * rpm / 60);
        }
        else {
            shooterMotor.setVelocity(0);
        }
    }
}
