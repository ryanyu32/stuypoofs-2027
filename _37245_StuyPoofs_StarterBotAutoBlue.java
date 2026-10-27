
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "_37245_StuyPoofs_StarterBotAutoBlue", group = "BioBuzz")
public class _37245_StuyPoofs_StarterBotAutoBlue extends LinearOpMode {

    private DcMotor lf, rf, lb, rb, intake;
    private DcMotorEx launcher;
    private IMU imu;
    private CRServo windmillServo;


    private static final double DRIVE_POWER = 0.30;
    private static final double FIRST_DRIVE_SECONDS = 1.5;
    private static final double SECOND_DRIVE_SECONDS = 1.2;
    private static final double THIRD_DRIVE_SECONDS = 1.2;
    private static final double LAUNCH_VELOCITY = 2000; //1250;
    private static final double FEED_SECONDS = 3.0;

    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, "left_front_drive");
        rf = hardwareMap.get(DcMotor.class, "right_front_drive");
        lb = hardwareMap.get(DcMotor.class, "left_back_drive");
        rb = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        imu = hardwareMap.get(IMU.class, "imu");


        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);

        stopDrive();

        imu.initialize(new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        ));

        telemetry.addLine("BLUE AUTO READY");
        telemetry.addLine("Check robot start position and IMU orientation.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        imu.resetYaw();

        driveForward(FIRST_DRIVE_SECONDS);
        turnDegrees(90);
    //   driveForward(SECOND_DRIVE_SECONDS);
    //    turnDegrees(-135);
    //    driveForward(THIRD_DRIVE_SECONDS);
    //    turnDegrees(-135);
        windmillServo.setPower(1.0);
        launcher.setVelocity(2000);
        sleep(1800);

        //intake.setPower(0.65);
        sleep((long) (FEED_SECONDS * 1000));

        //intake.setPower(0);
        launcher.setVelocity(0);
        windmillServo.setPower(0.0);
        stopDrive();
    }

    private void turnDegrees(double degrees) {
        double target = imu.getRobotYawPitchRollAngles()
                           .getYaw(AngleUnit.DEGREES) + degrees;
        double timeout = getRuntime() + 3.0;

        while (opModeIsActive() && getRuntime() < timeout) {
            double current = imu.getRobotYawPitchRollAngles()
                                 .getYaw(AngleUnit.DEGREES);
            double error = normalizeAngle(target - current);

            if (Math.abs(error) < 3.0) break;

            double power = Math.max(0.12,
                Math.min(0.35, Math.abs(error) * 0.012));
            setDrive(0, 0, Math.signum(error) * power);
        }

        stopDrive();
        sleep(200);
    }

    private void driveForward(double seconds) {
        setDrive(DRIVE_POWER, 0, 0);
        sleep((long) (seconds * 1000));
        stopDrive();
        sleep(150);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private void setDrive(double forward, double strafe, double turn) {
        double leftFront  = forward + strafe + turn;
        double rightFront = forward - strafe - turn;
        double leftBack   = forward - strafe + turn;
        double rightBack  = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(
            Math.max(Math.abs(leftFront), Math.abs(rightFront)),
            Math.max(Math.abs(leftBack), Math.abs(rightBack))
        ));

        lf.setPower(leftFront / max);
        rf.setPower(rightFront / max);
        lb.setPower(leftBack / max);
        rb.setPower(rightBack / max);
    }

    private void stopDrive() {
        setDrive(0, 0, 0);
    }
}
