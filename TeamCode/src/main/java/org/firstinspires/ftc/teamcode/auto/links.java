package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name = "links")
public final class links extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor i = null, s = null, st = null;
        Servo shootServo = null, blockServo = null;


        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDriveRR.class)) {
            MecanumDriveRR drive = new MecanumDriveRR(hardwareMap, beginPose);
            try { s = hardwareMap.dcMotor.get("shooter"); } catch (Exception e) { telemetry.addLine("Missing: shooter"); }
            try { st = hardwareMap.dcMotor.get("storage"); } catch (Exception e) { telemetry.addLine("Missing: storage"); }
            try { i = hardwareMap.dcMotor.get("intake"); } catch (Exception e) { telemetry.addLine("Missing: intake"); }
            try { shootServo = hardwareMap.servo.get("shoot_servo"); } catch (Exception e) { telemetry.addLine("Missing: shooting servo"); }
            try { blockServo = hardwareMap.servo.get("block_servo"); } catch (Exception e) { telemetry.addLine("Missing: blocking servo"); }


            if (s != null) s.setDirection(DcMotorSimple.Direction.REVERSE);
            if (st != null) st.setDirection(DcMotorSimple.Direction.FORWARD);



            waitForStart();
            Actions.runBlocking(
                    drive.actionBuilder(beginPose).strafeTo(new Vector2d(15,15)).build()
            );
        } else {
            throw new RuntimeException();
        }
    }
}
