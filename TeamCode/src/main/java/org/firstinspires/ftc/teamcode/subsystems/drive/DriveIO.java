package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.logging.Logger;

public class DriveIO {
    private final DcMotor fr, fl, br, bl;
    private final double exponent;
    private final IMU imu;

    public DriveIO(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, double exponent, IMU imu) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        this.exponent = exponent;
        this.imu = imu;
    }

    public void updateDrive(Gamepad gamepad1, Logger logger, int index) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        y = exponentialInput(y, this.exponent);
        x = exponentialInput(x, this.exponent);
        rx = exponentialInput(rx, this.exponent);

        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        logger.logValue(Double.toString(frontLeftPower), index);
        logger.logValue(Double.toString(backLeftPower), index + 1);
        logger.logValue(Double.toString(frontRightPower), index + 2);
        logger.logValue(Double.toString(backRightPower), index + 3);


        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);
    }

    private double exponentialInput(double value, double exponent) {
        return Math.copySign(Math.pow(Math.abs(value), exponent), value);
    }
}
