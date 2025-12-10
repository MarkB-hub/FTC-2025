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

@Autonomous(name = "3x red")
public final class ThreeRed extends LinearOpMode {

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
                    drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(-100, 0))
                            .turn(-43.7/180*Math.PI)
                            .strafeTo(new Vector2d(-100,35))
                            .build());

            st.setPower(0.2);
            i.setPower(0.6);
            sleep(1000);
            shootServo.setPosition(0);
            sleep(2000);
            st.setPower(0);
            i.setPower(0);
            blockServo.setPosition(0.5);
            sleep(1500);
        } else {
            throw new RuntimeException();
        }
    }
}
