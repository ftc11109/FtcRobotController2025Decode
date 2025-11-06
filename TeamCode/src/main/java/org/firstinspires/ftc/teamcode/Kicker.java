package org.firstinspires.ftc.teamcode;



import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Time;
import java.util.concurrent.TimeUnit;

public class Kicker {
    private final Gamepad gamepad;
    final ElapsedTime runtime;
    private boolean runningKicker;
    private long stopTime;
    final long motorTime = 125;
    private long motorStartTime = -motorTime;

    DcMotorEx kickerMotor;
    public Kicker(HardwareMap hardwareMap, Gamepad gamepad, ElapsedTime runTime) {
        this.runtime = runTime;
        this.gamepad = gamepad;
        kickerMotor = hardwareMap.get(DcMotorEx.class, "kicker_motor");
        PIDCoefficients pid = new PIDCoefficients(20, 3, 5);
        kickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kickerMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        kickerMotor.setTargetPosition(0);
        kickerMotor.setTargetPositionTolerance(5);
        kickerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        kickerMotor.setPower(1);
    }

    public void tick(double speed) {
//        if (gamepad.b && !this.runningKicker) {
//            this.motorStartTime = runtime.now(TimeUnit.MILLISECONDS);
//            this.runningKicker = true;
//            kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            kickerMotor.setPower(speed);
//        }
//        if (runtime.now(TimeUnit.MILLISECONDS) - this.motorStartTime > motorTime || kickerMotor.getCurrentPosition() >= 24) {
//            kickerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            kickerMotor.setPower(0);
//            this.stopTime = runtime.now(TimeUnit.MILLISECONDS);
//        }
//        if(runtime.now(TimeUnit.MILLISECONDS) + 1000 > this.stopTime) {
//            kickerMotor.setPower(0.4);
//            kickerMotor.setTargetPosition(0);
//        }
//        if((Math.abs(0 - kickerMotor.getCurrentPosition()) < 2 && kickerMotor.getTargetPosition() == 0) || runtime.now(TimeUnit.MILLISECONDS) > motorTime * 2) {
//            kickerMotor.setPower(0);
//            this.runningKicker = false;
//        }
        if(gamepad.right_bumper && !gamepad.rightBumperWasPressed() && !runningKicker) {
            kickerMotor.setTargetPosition(27);
            this.runningKicker = true;
        }
        else if(runningKicker && kickerMotor.getCurrentPosition() > 27) {
            kickerMotor.setTargetPosition(0);
            this.runningKicker = false;
        }
    }
//    public void goToPosition(int pos) {
//        kickerMotor.setTargetPosition(pos);
//        //kickerMotor.setTargetPosition(0);
//
//    }
}
