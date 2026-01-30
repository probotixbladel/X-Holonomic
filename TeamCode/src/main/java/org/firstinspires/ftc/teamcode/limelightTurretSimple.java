package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@Configurable
@TeleOp(name = "LimelightTurretVelocity")
public class limelightTurretSimple extends LinearOpMode {
    private Limelight3A limelight;
    private DcMotorEx turretMotor;
    public Pose blueGoal = new Pose(11, 135);
    public static double P = 10;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;
    //static double maxPos = 800;
    //static double minPos = 800;
    //boolean zeroed = false;

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


        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botPose = result.getBotpose_MT2();
                    double heading = result.getTx();
                    double robotX = botPose.getPosition().x;
                    double robotY = botPose.getPosition().y;


                    double dx = blueGoal.getX() - robotX;
                    double dy = blueGoal.getY() - robotY;


                    double angle = Math.atan2(dx, dy);

                    double deltaAngle = angle - heading;


                    turretMotor.setVelocity(deltaAngle);
                }
            }
        }
    }
}
