package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveIO;
import org.firstinspires.ftc.teamcode.subsystems.storage.IntakeIO;
import org.firstinspires.ftc.teamcode.subsystems.storage.StorageIO;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterIO;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionIO;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionLocalize;

@TeleOp
public class OpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fl = null, fr = null, bl = null, br = null, s1 = null, s2 = null, i = null, st = null;
        Servo shootServo = null, blockServo = null;
        WebcamName camera = null;

        try { fl = hardwareMap.dcMotor.get("fl"); } catch (Exception e) { telemetry.addLine("Missing: fl"); }
        try { fr = hardwareMap.dcMotor.get("fr"); } catch (Exception e) { telemetry.addLine("Missing: fr"); }
        try { bl = hardwareMap.dcMotor.get("bl"); } catch (Exception e) { telemetry.addLine("Missing: bl"); }
        try { br = hardwareMap.dcMotor.get("br"); } catch (Exception e) { telemetry.addLine("Missing: br"); }
        try { s1 = hardwareMap.dcMotor.get("shooter"); } catch (Exception e) { telemetry.addLine("Missing: shooter"); }
        try { s2 = hardwareMap.dcMotor.get("shooter1"); } catch (Exception e) { telemetry.addLine("Missing: shooter"); }

        try { st = hardwareMap.dcMotor.get("storage"); } catch (Exception e) { telemetry.addLine("Missing: storage"); }
        try { i = hardwareMap.dcMotor.get("intake"); } catch (Exception e) { telemetry.addLine("Missing: intake"); }
        try { shootServo = hardwareMap.servo.get("shoot_servo"); } catch (Exception e) { telemetry.addLine("Missing: shooting servo"); }
        try { blockServo = hardwareMap.servo.get("block_servo"); } catch (Exception e) { telemetry.addLine("Missing: blocking servo"); }

        try { camera = hardwareMap.get(WebcamName.class, "Webcam 1"); } catch (Exception e) { telemetry.addLine("Missing: Webcam 1"); }

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.initialize(parameters);

        telemetry.update();
        if (fl != null) fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (fr != null) fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (bl != null) bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (br != null) br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (bl != null) bl.setDirection(DcMotorSimple.Direction.FORWARD);
        if (br != null) br.setDirection(DcMotorSimple.Direction.FORWARD);
        if (fl != null) fl.setDirection(DcMotorSimple.Direction.REVERSE);
        if (fr != null) fr.setDirection(DcMotorSimple.Direction.FORWARD);
        if (s1 != null) s1.setDirection(DcMotorSimple.Direction.FORWARD);
        if (s2 != null) s2.setDirection(DcMotorSimple.Direction.REVERSE);

        if (st != null) st.setDirection(DcMotorSimple.Direction.FORWARD);

        DriveIO drive = (fl != null && fr != null && bl != null && br != null) ? new DriveIO(fl, fr, bl, br, Constants.DRIVE_CONTROLLER_EXPONENT, imu) : null;

        ShooterIO shooter = (s1 != null) && (s2 != null) ? new ShooterIO(s1,s2) : null;
        if (shooter != null) {shooter.setRampRate(Constants.SHOOTER_RAMP_RATE); shooter.setShootingPower(Constants.DEFAULT_SHOOTER_SPEED);}

        StorageIO storage = (st != null) ? new StorageIO(st) : null;

        IntakeIO intake = (i != null) ? new IntakeIO(i) : null;

        VisionIO vision = (camera != null) ? new VisionIO(camera) : null;

        VisionLocalize localize = (vision != null) ? new VisionLocalize(vision) : null;

        waitForStart();

        if (isStopRequested()) return;
        boolean blocking = false;
        blockServo.setPosition(0);
        while (opModeIsActive()) {
            if (drive != null) drive.updateDrive(gamepad1);
            if (intake != null) intake.updateIntake(gamepad1);
            if (storage != null) storage.updateStorage(gamepad1);
            if (shooter != null) shooter.updateShooter(gamepad1);

            if (localize != null) localize.update();


            if(gamepad1.leftBumperWasPressed()) {
                if (blocking) {
                    blockServo.setPosition(0);
                } else {
                    blockServo.setPosition(0.5);
                }
                blocking = !blocking;
            }

            if (gamepad1.right_bumper) {
                shootServo.setPosition(0);

            } else {
                shootServo.setPosition(0.4);
            }


            telemetry.addLine(imu.getRobotYawPitchRollAngles().toString());
            telemetry.update();
        }

        if (vision != null) vision.stop();
    }
}
