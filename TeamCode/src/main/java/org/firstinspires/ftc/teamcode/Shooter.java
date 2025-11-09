package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private final Gamepad gamepad;
    DcMotorEx shooterMotor;
    public double targetTps;

    public Shooter(HardwareMap hardwareMap, Gamepad gamepad, Gate gate) {
        this.gamepad = gamepad;
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void tick(int rpm, int rpm2, double gearing) {
        if(gamepad.x) {
            shooterMotor.setVelocity(0);
            this.targetTps = 0;
        }
        else if(gamepad.a) {
            //Positive is clockwise direction
            shooterMotor.setVelocity((double) (28 * (rpm / gearing)) / 60);
            this.targetTps = (rpm / gearing) * 28 / 60;
            //shooterMotor.setVelocity(28 * 5000 / 60);
        }
        else if(gamepad.y) {
            shooterMotor.setVelocity((28 * (rpm2 / gearing)) / 60);
            this.targetTps = (rpm2 / gearing) * 28 / 60;
        }
    }
}
