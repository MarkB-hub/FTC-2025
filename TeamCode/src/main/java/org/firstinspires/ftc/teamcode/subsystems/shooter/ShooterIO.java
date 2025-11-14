package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ShooterIO {
    private DcMotor s1, s2;

    private double targetPower = 0.0;
    private double currentPower = 0.0;
    private double maxRampRate = 0.05;
    private boolean shooting = true;

    public ShooterIO(DcMotor s1) {
        this.s1 = s1;
//        this.s2 = s2;
//        this.s2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void updateShooter(Gamepad gamepad1) {
        if (gamepad1.dpadUpWasPressed()) {
            shooting = !shooting;
        }
        if (shooting) {
            targetPower = 0.59;
        } else {
            targetPower = 0.0;
        }

        if (currentPower < targetPower) {
            currentPower += maxRampRate;
            if (currentPower > targetPower) currentPower = targetPower;
        } else if (currentPower > targetPower) {
            currentPower -= maxRampRate;
            if (currentPower < targetPower) currentPower = targetPower;
        }

        s1.setPower(currentPower);
//        s2.setPower(currentPower);
    }

    public void setRampRate(double rampRate) {
        this.maxRampRate = rampRate;
    }
}
