package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class IntakeIO {
    private DcMotor i;

    private double targetPower = 0.0;


    public IntakeIO(DcMotor i) {
        this.i = i;
    }

    public void updateIntake(Gamepad gamepad1) {
        if (gamepad1.a) {
            targetPower = 1;
        } else if (gamepad1.y) {
            targetPower = -1;
        } else {
            targetPower = 0.0;
        }
        i.setPower(targetPower);
    }

    public void setPower(double power) {
        i.setPower(power);
    }
}
