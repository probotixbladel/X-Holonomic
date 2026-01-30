package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Configurable
@TeleOp(name = "LimelightTurretVelocity")
public class limelightTurretBS extends LinearOpMode {

    private Pose3D botPose;
    public Pose blueGoal = new Pose(11, 135);

    private Limelight3A limelight;
    private DcMotorEx turretMotor;
    private RevTouchSensor zeroSwitch;

    private TelemetryManager telemetryM;

    public static double P = 10;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;
    static double maxPos = 800;
    static double minPos = 800;
    boolean zeroed = false;

    @Override
    public void runOpMode() {


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        zeroSwitch = hardwareMap.get(RevTouchSensor.class, "zeroSwitch");
        turretMotor.setDirection(DcMotor.Direction.FORWARD);

        limelight.start();
        limelight.pipelineSwitch(0);

        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, I, D, F));


        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                botPose = result.getBotpose();
                double heading = Math.toRadians(botPose.getOrientation().getYaw());
                double robotX = botPose.getPosition().x;
                double robotY = botPose.getPosition().y;
                //double tx = result.getTx();

                double dx = blueGoal.getX() - robotX;
                double dy = blueGoal.getY() - robotY;


                double angle = Math.atan2(dx, dy);


                //double angle = Math.abs(Math.atan((botpose.getPosition().x - blueGoal.getX())/(botpose.getPosition().y - blueGoal.getY())) - 0.5 * Math.PI);
                double deltaAngle = angle - heading;

                //int targetVelocity = deltaAngle * ;
                turretMotor.setVelocity(deltaAngle);


            }

            /*
            if (zeroSwitch.getState()) {
                turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                zeroed = true;

                telemetryM.debug("zeroed", zeroed);
            }
            else if (!zeroSwitch.getState()) {
                zeroed = false;

            }
            */

        }
    }
}
