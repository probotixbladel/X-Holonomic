package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "LimelightTurretVelocity")
public class limelightTurretBS extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotorEx turretMotor;

    public static double P = 10;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;





    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, I, D, F));

        int maxVelocityTicksPerSec = 2000;

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                double tx = result.getTx();

                double kP = maxVelocityTicksPerSec / 27.0;
                double targetVelocity = tx * kP;

                targetVelocity = Math.max(-maxVelocityTicksPerSec,
                        Math.min(maxVelocityTicksPerSec, targetVelocity));

                turretMotor.setVelocity(targetVelocity);

                telemetry.addData("TX", tx);
                telemetry.addData("Target Velocity (ticks/sec)", targetVelocity);

            } else {
                turretMotor.setVelocity(0);
                telemetry.addData("Limelight", "No Targets");
            }

            telemetry.update();
        }
    }
}
