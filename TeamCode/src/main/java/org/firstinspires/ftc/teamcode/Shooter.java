package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private final Gamepad gamepad;
    private boolean isAutonomous;
    DcMotorEx shooterMotor;
    public double targetTps;
    public int closeSpeed = 2850;
    public int farSpeed = 5000;
    public double gearing = 1;

    public Shooter(HardwareMap hardwareMap, Gamepad gamepad, Gate gate, boolean isAutonomous) {
        this.gamepad = gamepad;
        this.isAutonomous = isAutonomous;
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void tick() {
        if(gamepad.x) {
            shooterMotor.setVelocity(0);
            this.targetTps = 0;
        }
        else if(gamepad.a) {
            //Positive is clockwise direction
            shooterMotor.setVelocity((double) (28 * (closeSpeed / gearing)) / 60);
            this.targetTps = (28 * (closeSpeed / gearing)) / 60;
            //shooterMotor.setVelocity(28 * 5000 / 60);
        }
        else if(gamepad.y) {
            shooterMotor.setVelocity((28 * (farSpeed / gearing)) / 60);
            this.targetTps = (28 * (farSpeed / gearing)) / 60;
        }
    }
    //Start the shooter at close-range speed
    public void startClose() {
        if(this.isAutonomous) {
            shooterMotor.setVelocity((28 * (closeSpeed / gearing) / 60));
            this.targetTps = (28 * (closeSpeed / gearing)) / 60;
        }
    }
    //Start the shooter at far-range speed
    public void startFar() {
        if(this.isAutonomous) {
            shooterMotor.setVelocity((28 * (farSpeed / gearing) / 60));
            this.targetTps = (28 * (farSpeed / gearing)) / 60;
        }
    }
    //Stop the shooter
    public void stopMotor() {
        if(this.isAutonomous) {
            shooterMotor.setVelocity(0);
            this.targetTps = 0;
        }
    }
}
