package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "_37245_SBotAutoV2", group = "BioBuzz")
public class _37245_SBotAutoV2 extends LinearOpMode {

    private DcMotor lf, rf, lb, rb, intake;
    private DcMotorEx launcher;
    private IMU imu;
    private CRServo windmill;

    private static final double DRIVE_POWER = 0.45;
    private static final double STRAFE_SECONDS = 1.75;
    private static final double FIRST_DRIVE_SECONDS = 3.071;
    private static final double SECOND_DRIVE_SECONDS = 0.3; // final strafe
    // private static final double THIRD_DRIVE_SECONDS = 1.2;
    private static final double LAUNCH_VELOCITY_TARGET = 1180;
    private static final double LAUNCH_VELOCITY_MIN = 1100;
    private static final double SPIN_UP_TIMEOUT_SECONDS = 4.0;
    private static final double FEED_SECONDS = 10.0;

    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, "left_front_drive");
        rf = hardwareMap.get(DcMotor.class, "right_front_drive");
        lb = hardwareMap.get(DcMotor.class, "left_back_drive");
        rb = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        imu = hardwareMap.get(IMU.class, "imu");
        windmill = hardwareMap.get(CRServo.class, "windmillServo");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        // --- Spin up launcher (wait until fast enough, with a timeout) ---
        launcher.setVelocity(LAUNCH_VELOCITY_TARGET);

        double spinUpDeadline = getRuntime() + SPIN_UP_TIMEOUT_SECONDS;
        while (opModeIsActive()
                && Math.abs(launcher.getVelocity()) < LAUNCH_VELOCITY_MIN
                && getRuntime() < spinUpDeadline) {
            telemetry.addData("launcher vel", launcher.getVelocity());
            telemetry.update();
            idle();
        }

        // --- Feed for FEED_SECONDS (Stop button still works) ---
        windmill.setPower(-1);
        // intake.setPower(1.0); // uncomment if balls need the intake to reach the windmill

        double feedEnd = getRuntime() + FEED_SECONDS;
        while (opModeIsActive() && getRuntime() < feedEnd) {
            telemetry.addData("launcher vel", launcher.getVelocity());
            telemetry.update();
            idle();
        }

        launcher.setVelocity(0);
        windmill.setPower(0.0);
        intake.setPower(0.0);

        // --- Drive off the line ---
        strafe(DRIVE_POWER, STRAFE_SECONDS);
        driveBackward(FIRST_DRIVE_SECONDS);
        // turnDegrees(-95);
        // driveForward(SECOND_DRIVE_SECONDS);
        strafe(DRIVE_POWER, SECOND_DRIVE_SECONDS);

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

            telemetry.addData("yaw", current);
            telemetry.addData("error", error);
            telemetry.update();

            if (Math.abs(error) < 3.0) break;

            double power = Math.max(0.12,
                Math.min(0.15, Math.abs(error) * 0.012));
            // IMU yaw is CCW-positive, but positive turn in setDrive is CW, so negate
            setDrive(0, 0, -Math.signum(error) * power);
        }

        stopDrive();
        sleep(200);
    }

    private void strafe(double power, double seconds) {
        setDrive(0, power, 0);
        sleep((long) (seconds * 1000));
        stopDrive();
        sleep(100);
    }

    private void driveBackward(double seconds) {
        setDrive(-DRIVE_POWER, 0, 0);
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
