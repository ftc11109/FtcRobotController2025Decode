package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class Shooter {
    private final Gamepad gamepad;
    DcMotorEx shooterMotor;
    private ElapsedTime runtime;
    public double targetTps;
    public int closeSpeed = 2850;
    public int farSpeed = 3700;
    public int medSpeed = 2525;
    public double gearing = 1;
    public long revStartTime = -500;
    public boolean reversing = false;

    public Shooter(HardwareMap hardwareMap, Gamepad gamepad, Gate gate, ElapsedTime runtime) {
        this.gamepad = gamepad;
        this.runtime = runtime;
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
            startMed();
        }
        else if(gamepad.dpad_down) {
            this.revStartTime = runtime.now(TimeUnit.MILLISECONDS);
            this.reversing = true;
            this.startCustom(-3000);
        }
        if(this.reversing && this.revStartTime + 250 < runtime.now(TimeUnit.MILLISECONDS)) {
            this.reversing = false;
            this.stopMotor();
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
