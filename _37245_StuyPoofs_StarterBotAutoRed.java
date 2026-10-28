
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(
    name = "_37245_StuyPoofs_StarterBotAutoRed",
    group = "BioBuzz"
)
public class _37245_StuyPoofs_StarterBotAutoRed extends LinearOpMode {

    private DcMotor lf, rf, lb, rb;
    private DcMotor intake;
    private DcMotorEx launcher;

    private CRServo windmillServo;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    private IMU imu;

    private static final double DRIVE_POWER = 0.30;
    private static final double FIRST_DRIVE_SECONDS = 1.5;

    // Tune these times during field testing.
    private static final double COLLECTION_DRIVE_SECONDS = 2.0;
    private static final double INTAKE_POWER = 1.0;

    // Match the team's TeleOp launcher configuration.
    private static final double LAUNCHER_TARGET_VELOCITY = 1100;
    private static final double SPINUP_SECONDS = 1.8;
    private static final double FEED_SECONDS = 3.0;

    @Override
    public void runOpMode() {

        // Mecanum drivetrain.
        lf = hardwareMap.get(DcMotor.class, "left_front_drive");
        rf = hardwareMap.get(DcMotor.class, "right_front_drive");
        lb = hardwareMap.get(DcMotor.class, "left_back_drive");
        rb = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Mechanisms: names match the working TeleOp.
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        imu = hardwareMap.get(IMU.class, "imu");

        // Match the TeleOp motor directions.
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Match the TeleOp launcher setup.
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            new PIDFCoefficients(40, 0, 0, 12.5)
        );

        // Match the TeleOp servo directions.
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        stopEverything();

        // Must match the physical IMU mounting orientation.
        imu.initialize(new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        ));

        telemetry.addLine("RED AUTO READY");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            stopEverything();
            return;
        }

        imu.resetYaw();

        // 1. Move away from the starting position.
        driveForward(FIRST_DRIVE_SECONDS);

        if (!opModeIsActive()) {
            stopEverything();
            return;
        }

        // 2. Turn toward the Hive.
        turnDegrees(-90);

        if (!opModeIsActive()) {
            stopEverything();
            return;
        }

        // 3. Shoot the four preloaded balls.
        shootBalls();

        if (!opModeIsActive()) {
            stopEverything();
            return;
        }

        // 4. Turn another 90 degrees toward the collection area.
        turnDegrees(-90);

        if (!opModeIsActive()) {
            stopEverything();
            return;
        }

        // 5. Drive forward while collecting balls.
        intake.setPower(INTAKE_POWER);
        leftIntakeServo.setPower(INTAKE_POWER);
        rightIntakeServo.setPower(INTAKE_POWER);

        driveForward(COLLECTION_DRIVE_SECONDS);

        // Stop collection before retracing the route.
        stopIntake();

        if (!opModeIsActive()) {
            stopEverything();
            return;
        }

        // 6. Drive backward for approximately the same distance.
        driveBackward(COLLECTION_DRIVE_SECONDS);

        if (!opModeIsActive()) {
            stopEverything();
            return;
        }

        // 7. Turn 90 degrees the opposite way to face the Hive again.
        turnDegrees(90);

        if (!opModeIsActive()) {
            stopEverything();
            return;
        }

        // 8. Shoot the newly collected balls.
        shootBalls();

        // 9. Finish safely.
        stopEverything();

        telemetry.addLine("AUTO COMPLETE");
        telemetry.update();
    }

    private void shootBalls() {

        if (!opModeIsActive()) return;

        // Spin up launcher.
        windmillServo.setPower(0);
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

        telemetry.addLine("LAUNCHER SPINNING UP");
        telemetry.update();

        sleep((long) (SPINUP_SECONDS * 1000));

        if (!opModeIsActive()) {
            stopEverything();
            return;
        }

        // Feed balls. Uses timed feeding rather than the TeleOp's
        // 1180 velocity threshold, which exceeds the 1100 target.
        telemetry.addLine("FEEDING BALLS");
        telemetry.addData("Launcher velocity", launcher.getVelocity());
        telemetry.update();

        windmillServo.setPower(1.0);

        sleep((long) (FEED_SECONDS * 1000));

        windmillServo.setPower(0);
        launcher.setVelocity(0);
    }

    private void driveForward(double seconds) {
        setDrive(DRIVE_POWER, 0, 0);
        sleep((long) (seconds * 1000));
        stopDrive();
        sleep(150);
    }

    private void driveBackward(double seconds) {
        setDrive(-DRIVE_POWER, 0, 0);
        sleep((long) (seconds * 1000));
        stopDrive();
        sleep(150);
    }

    private void turnDegrees(double degrees) {

        double target = imu.getRobotYawPitchRollAngles()
            .getYaw(AngleUnit.DEGREES) + degrees;

        double timeout = getRuntime() + 3.0;

        while (opModeIsActive() && getRuntime() < timeout) {

            double current = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);

            double error = normalizeAngle(target - current);

            telemetry.addData("Turn error", error);
            telemetry.update();

            if (Math.abs(error) < 3.0) {
                break;
            }

            double power = Math.max(
                0.12,
                Math.min(0.35, Math.abs(error) * 0.012)
            );

            setDrive(0, 0, Math.signum(error) * power);
        }

        stopDrive();
        sleep(200);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // Same mecanum power calculation as the website TeleOp.
    private void setDrive(double forward, double strafe, double turn) {

        double leftFront = forward + strafe + turn;
        double rightFront = forward - strafe - turn;
        double leftBack = forward - strafe + turn;
        double rightBack = forward + strafe - turn;

        double max = Math.max(
            1.0,
            Math.max(
                Math.max(Math.abs(leftFront), Math.abs(rightFront)),
                Math.max(Math.abs(leftBack), Math.abs(rightBack))
            )
        );

        lf.setPower(leftFront / max);
        rf.setPower(rightFront / max);
        lb.setPower(leftBack / max);
        rb.setPower(rightBack / max);
    }

    private void stopDrive() {
        setDrive(0, 0, 0);
    }

    private void stopIntake() {
        intake.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }

    private void stopEverything() {
        stopDrive();
        stopIntake();

        if (windmillServo != null) {
            windmillServo.setPower(0);
        }

        if (launcher != null) {
            launcher.setVelocity(0);
        }
    }
}
