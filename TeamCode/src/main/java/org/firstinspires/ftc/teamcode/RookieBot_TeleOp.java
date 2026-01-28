package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="RookieBot", group="Linear OpMode")
public class RookieBot_TeleOp extends LinearOpMode {

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    @Override
    public void runOpMode() {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront"); //0
        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); //1
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");//2
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");//3
        waitForStart();
        while (opModeIsActive()) {
            double leftPower;
            double rightPower;

            double drive = -gamepad1.right_stick_x;
            double turn = gamepad1.left_stick_y;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);

            rightFront.setPower(rightPower);
            rightRear.setPower(rightPower);
            leftFront.setPower(leftPower);
            leftRear.setPower(leftPower);
        }

    }
}
