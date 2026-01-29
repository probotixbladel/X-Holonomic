package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Configurable
@TeleOp(name = "LimelightTurretVelocity")
public class limelightTurretBS extends LinearOpMode {

    private Pose3D botpose;
    public Pose blueGoal = new Pose(11,135);

    private Limelight3A limelight;
    private DcMotorEx turretMotor;
    private TelemetryManager telemetryM;

    public static double P = 10;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;





    @Override
    public void runOpMode() {


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotor.Direction.FORWARD);

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

                botpose = result.getBotpose();
                double heading = botpose.getOrientation().getYaw();
                double robotX = botpose.getPosition().x;
                double robotY = botpose.getPosition().y;

                double angle = Math.abs(Math.atan((robotX - blueGoal.getX())/(robotY - blueGoal.getY())) - 0.5 * Math.PI);
                double deltaAngle = heading - angle;

                double tx = result.getTx();

                double kP = -maxVelocityTicksPerSec / 27.0;
                double targetVelocity = deltaAngle * kP;

                targetVelocity = Math.max(-maxVelocityTicksPerSec,
                        Math.min(maxVelocityTicksPerSec, targetVelocity));

                turretMotor.setVelocity(targetVelocity);

                telemetry.addData("dAngle", deltaAngle);
                telemetry.addData("Angle", angle);
                telemetry.addData("Target Velocity (ticks/sec)", targetVelocity);

            } else {
                turretMotor.setVelocity(0);
                telemetry.addData("Limelight", "No Targets");
            }

            telemetry.update();
        }
    }
}
