package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private final Gamepad gamepad;
    DcMotorEx shooterMotor;
    public double targetTps;
    public int closeSpeed = 2850;
    public int farSpeed = 5000;
    public int medSpeed = 2525;
    public double gearing = 1;

    public Shooter(HardwareMap hardwareMap, Gamepad gamepad, Gate gate) {
        this.gamepad = gamepad;
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void tick() {
        if(gamepad.x) {
            this.stopMotor();
        }
        else if(gamepad.a) {
            this.startClose();
        }
        else if(gamepad.y) {
            this.startFar();
        }
        else if(gamepad.b) {
            startCustom(medSpeed);
        }
    }
    //Start the shooter at close-range speed
    public void startClose() {
        shooterMotor.setVelocity((28 * (closeSpeed / gearing) / 60));
        this.targetTps = (28 * (closeSpeed / gearing)) / 60;
    }
    //Start the shooter at far-range speed
    public void startFar() {
        shooterMotor.setVelocity((28 * (farSpeed / gearing) / 60));
        this.targetTps = (28 * (farSpeed / gearing)) / 60;
    }
    public void startMed() {
        shooterMotor.setVelocity((28 * (medSpeed / gearing) / 60));
        this.targetTps = (28 * (medSpeed / gearing)) / 60;
    }
    //Stop the shooter
    public void stopMotor() {
        shooterMotor.setVelocity(0);
        this.targetTps = 0;
    }
    public void startCustom(int rpm) {
        shooterMotor.setVelocity((28 * (rpm / gearing) / 60));
        this.targetTps = (28 * (rpm / gearing)) / 60;
    }
}
