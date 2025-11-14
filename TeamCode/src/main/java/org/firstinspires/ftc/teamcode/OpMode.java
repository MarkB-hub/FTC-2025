package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.logging.Logger;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveIO;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIO;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterIO;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionIO;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionLocalize;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp
public class OpMode extends LinearOpMode {
    public int logIndex = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fl = null, fr = null, bl = null, br = null, s = null, i = null;
        WebcamName camera = null;

        try { fl = hardwareMap.dcMotor.get("fl"); } catch (Exception e) { telemetry.addLine("Missing: fl"); }
        try { fr = hardwareMap.dcMotor.get("fr"); } catch (Exception e) { telemetry.addLine("Missing: fr"); }
        try { bl = hardwareMap.dcMotor.get("bl"); } catch (Exception e) { telemetry.addLine("Missing: bl"); }
        try { br = hardwareMap.dcMotor.get("br"); } catch (Exception e) { telemetry.addLine("Missing: br"); }
        try { s = hardwareMap.dcMotor.get("shooter"); } catch (Exception e) { telemetry.addLine("Missing: shooter"); }
        try { i = hardwareMap.dcMotor.get("intake"); } catch (Exception e) { telemetry.addLine("Missing: intake"); }
        try { camera = hardwareMap.get(WebcamName.class, "Webcam 1"); } catch (Exception e) { telemetry.addLine("Missing: Webcam 1"); }

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
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
        if (s != null) s.setDirection(DcMotorSimple.Direction.REVERSE);

        DriveIO drive = (fl != null && fr != null && bl != null && br != null)
                ? new DriveIO(fl, fr, bl, br, 1.5, imu)
                : null;

        ShooterIO shooter = (s != null)
                ? new ShooterIO(s)
                : null;
        if (shooter != null) shooter.setRampRate(0.005);

        IntakeIO intake = (i != null)
                ? new IntakeIO(i)
                : null;

        VisionIO vision = (camera != null)
                ? new VisionIO(camera)
                : null;
        VisionLocalize localize = (vision != null)
                ? new VisionLocalize(vision)
                : null;

        telemetry.addLine("Initialization Complete — waiting for start.");
        telemetry.update();

        Logger logger = new Logger(telemetry);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            logIndex = 0;
            // Updating
            if (drive != null) drive.updateDrive(gamepad1, logger, logIndex);
            logIndex += 4;
            if (shooter != null) shooter.updateShooter(gamepad1);
            if (intake != null) intake.updateIntake(gamepad1);
            if (localize != null) localize.update();

            logger.logValue(imu.getRobotYawPitchRollAngles().toString(), logIndex);
            logIndex ++;

            // Logging
            if (vision != null && vision.isActive()) {
                List<AprilTagDetection> detections = vision.getDetections();
                if (detections != null && !detections.isEmpty()) {
                    AprilTagDetection firstTag = detections.get(0);
                    logger.logValue("Tag: " + firstTag.id, logIndex);
                    logger.logValue("Pose: " + localize.getLastPose(), logIndex + 1);
                } else {
                    logger.logValue("Tag: None", 0);
                    logger.logValue("Pose: " + localize.getLastPose(), logIndex + 1);
                }
                logIndex += 2;
            }




            logger.update();
        }

        if (vision != null) vision.stop();
    }
}
