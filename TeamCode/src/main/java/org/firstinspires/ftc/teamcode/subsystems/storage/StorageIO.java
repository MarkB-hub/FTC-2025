package org.firstinspires.ftc.teamcode.subsystems.storage;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class StorageIO {
    private DcMotor i;

    private double targetPower = 0.0;


    public StorageIO(DcMotor i) {
        this.i = i;
    }

    public void updateStorage(Gamepad gamepad1) {
        if (gamepad1.b) {
            targetPower = 1;
        } else if (gamepad1.y) {
            targetPower = -1;
        } else {
            targetPower = 0.0;
        }
        i.setPower(targetPower);
    }
}
