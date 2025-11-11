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
    public String state = "IDLE";
    private Gate gate;

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
        if (gamepad.right_bumper && !gamepad.rightBumperWasPressed() && this.state == "IDLE" && !this.isAutonomous) {
            this.motorStartTime = runtime.now(TimeUnit.MILLISECONDS);
            this.state = "KICKING";
            kickerMotor.setPower(1);
        }
        if(runtime.now(TimeUnit.MILLISECONDS) > this.motorStartTime + 250 && this.state == "KICKING") {
            this.state = "RETURNING";
            kickerMotor.setPower(-1);
            this.motorStartTime = runtime.now(TimeUnit.MILLISECONDS);
        }
        if(this.runtime.now(TimeUnit.MILLISECONDS) > this.motorStartTime + 270 && this.state == "RETURNING") {
            this.state = "IDLE";
            kickerMotor.setPower(-0.02);
        }

    }
    public void kick() {
        if(isAutonomous) {
            if (this.state == "IDLE") {
                this.motorStartTime = runtime.now(TimeUnit.MILLISECONDS);
                this.state = "KICKING";
                kickerMotor.setPower(1);
            }
        }
    }
}
