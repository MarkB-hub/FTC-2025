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

@Autonomous(name = "6x blue")
public final class SixBlue extends LinearOpMode {

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
            blockServo.setPosition(0);

            s.setPower(Constants.DEFAULT_SHOOTER_SPEED);
            Actions.runBlocking(
                    drive.actionBuilder(beginPose).strafeTo(new Vector2d(5,0)).build()
            );
            sleep(1000);
            st.setPower(0.2);
            i.setPower(0.6);
            sleep(600);
            shootServo.setPosition(0);
            sleep(2000);
            st.setPower(0);
            i.setPower(0);

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(33,0))
                            .turn((-46.3-90)*Math.PI/180)
                            .build());
            MecanumDriveRR ball_set_2 = new MecanumDriveRR(hardwareMap, beginPose);
            Actions.runBlocking(
                    ball_set_2.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(0,40))
                            .build());
            st.setPower(1);
            i.setPower(1);
            shootServo.setPosition(0.4);
            blockServo.setPosition(0.5);
            Actions.runBlocking(
                    ball_set_2.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(30,0))
                            .build());
            st.setPower(0);
            i.setPower(0);
            blockServo.setPosition(0);

            Actions.runBlocking(
                    ball_set_2.actionBuilder(beginPose)
                            .turn((43.3+90)*Math.PI/180)
                            .strafeTo(new Vector2d(40,-40))
                            .build());
//            MecanumDriveRR beun = new MecanumDriveRR(hardwareMap, beginPose);
//
//            Actions.runBlocking(
//                    beun.actionBuilder(beginPose)
//                            .strafeTo(new Vector2d(0,10))
//                            .build());
            sleep(500);
            st.setPower(0.2);
            i.setPower(0.6);
            sleep(600);
            shootServo.setPosition(0);
            sleep(2000);
        } else {
            throw new RuntimeException();
        }
    }
}
