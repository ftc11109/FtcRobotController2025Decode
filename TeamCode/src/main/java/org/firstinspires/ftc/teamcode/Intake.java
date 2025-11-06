package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final Gamepad gamepad;
    DcMotor intakeMotor;

    public Intake(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void tick(double speed) {
        if(gamepad.right_trigger > 0.25) {
            //in
            //counterclockwise
            intakeMotor.setPower(speed);
        }
        else if(gamepad.right_bumper) {
            //out
            //clockwise
            intakeMotor.setPower(-speed);
        }
        else {
            intakeMotor.setPower(0);
        }
    }
}
