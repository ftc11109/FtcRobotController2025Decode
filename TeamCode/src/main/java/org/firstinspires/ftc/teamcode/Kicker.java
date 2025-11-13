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
    private boolean isAutonomous;
    final long motorTime = 125;
    private long motorStartTime = -motorTime;
    public enum STATE {
        CLOSING,
        KICKING,
        RETURNING,
        IDLE,
        OPENING,
        RELOADING
    };
    public STATE state = STATE.IDLE;
    private Gate gate;

    public final long servoTime = 500;
    public final long kickingTime = 250;
    public final long reloadingTime = 1000;
    public final long totalTime = servoTime * 2 + kickingTime * 2 + reloadingTime + 1000;

    DcMotorEx kickerMotor;
    public Kicker(HardwareMap hardwareMap, Gamepad gamepad, ElapsedTime runTime, boolean isAutonomous, Gate gate) {
        this.runtime = runTime;
        this.gamepad = gamepad;
        this.gate = gate;
        this.isAutonomous = isAutonomous;
        kickerMotor = hardwareMap.get(DcMotorEx.class, "kicker_motor");
        PIDCoefficients pid = new PIDCoefficients(20, 3, 5);
        kickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kickerMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        kickerMotor.setTargetPosition(0);
        kickerMotor.setTargetPositionTolerance(5);
        kickerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //kickerMotor.setPower(1);
    }

    public void tick() {
        if(!this.isAutonomous) {
            if (gamepad.right_bumper && !gamepad.rightBumperWasPressed() && this.state == STATE.IDLE) {
                this.motorStartTime = runtime.now(TimeUnit.MILLISECONDS);
                this.state = STATE.CLOSING;
                gate.gateUp();
            }
        }
        if(runtime.now(TimeUnit.MILLISECONDS) > this.motorStartTime + servoTime + 1000 && this.state == STATE.CLOSING) {
            this.state = STATE.KICKING;
            this.motorStartTime = runtime.now(TimeUnit.MILLISECONDS);
            kickerMotor.setPower(1);
        }
        if(runtime.now(TimeUnit.MILLISECONDS) > this.motorStartTime + 250 && this.state == STATE.KICKING) {
            this.state = STATE.RETURNING;
            kickerMotor.setPower(-1);
            this.motorStartTime = runtime.now(TimeUnit.MILLISECONDS);
        }
        if(this.runtime.now(TimeUnit.MILLISECONDS) > this.motorStartTime + 270 && this.state == STATE.RETURNING) {
            this.state = STATE.OPENING;
            this.motorStartTime = runtime.now(TimeUnit.MILLISECONDS);
            gate.gateDown();
            kickerMotor.setPower(-0.02);
        }
        if(this.runtime.now(TimeUnit.MILLISECONDS) > this.motorStartTime + servoTime && this.state == STATE.OPENING) {
            this.state = STATE.RELOADING;
            this.motorStartTime = runtime.now(TimeUnit.MILLISECONDS);
        }
        if(this.runtime.now(TimeUnit.MILLISECONDS) > this.motorStartTime + reloadingTime && this.state == STATE.RELOADING) {
            this.state = STATE.IDLE;
        }

    }
    public void kick() {
        if(isAutonomous) {
            if (this.state == STATE.IDLE) {
                this.motorStartTime = runtime.now(TimeUnit.MILLISECONDS);
                this.state = STATE.CLOSING;
                gate.gateUp();
            }
        }
    }
}
