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
    private double shootingPower;

    public ShooterIO(DcMotor s1, double shootingPower) {
        this.s1 = s1;
        this.shootingPower = shootingPower;
//        this.s2 = s2;
//        this.s2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void updateShooter(Gamepad gamepad1) {
        if (gamepad1.dpadUpWasPressed()) {
            shooting = !shooting;
        }
        if (shooting) {
            targetPower = shootingPower;
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
