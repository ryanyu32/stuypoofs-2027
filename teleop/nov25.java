package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//imports for huskylens
import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "StuyPoofsFullSRC", group = "Bot")
//@Disabled
public class StuyPoofsSRCv1 extends OpMode {

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotorEx leftShooter = null;
    private DcMotorEx rightShooter = null;

    private DcMotorEx leftIntake = null;
    private DcMotorEx rightIntake = null;

    private Servo leftPlowerServo = null;
    private Servo rightPlowerServo = null;
    private Servo indexerServo = null;
    
    private IMU imu = null;

    // need to update
    public final int LAUNCHER_TARGET_VELOCITY = 1550; //needs to be higher 
    public final int LAUNCHER_MIN_VELOCITY = 1500;
    public final int LAUNCHER_COAST_VELOCITY = 1400;
    public final int INDEXER_FEED_VELOCITY = 2500;
    
    //current logic is for a position; not for power
    double plowerPosition;
    // double blockerPower;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double intakePower;
    

    //ran at INIT, this is just setup
    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "eh1");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "ch1");
        leftBackDrive = hardwareMap.get(DcMotor.class, "eh0");
        rightBackDrive = hardwareMap.get(DcMotor.class, "ch0");

        leftShooter = hardwareMap.get(DcMotorEx.class, "eh2");
        rightShooter = hardwareMap.get(DcMotorEx.class, "ch2");

        leftIntake = hardwareMap.get(DcMotorEx.class, "eh3");
        rightIntake = hardwareMap.get(DcMotorEx.class, "ch3");
        
        leftPlowerServo = hardwareMap.get(Servo.class, "ehs0");
        rightPlowerServo = hardwareMap.get(Servo.class, "chs0");
        indexerServo = hardwareMap.get(Servo.class, "eh5");
        // blockerServo = hardwareMap.get(CRServo.class, "");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(//UPDATE
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        );
        imu.initialize(parameters);

        /*
        ch0 = back right wheel, positive
        ch1 = front right wheel, positive
        ch2 = right shooter, negative
        ch3 = right intake, negative
        eh0 = back left wheel, positive
        eh1 = front left, negative
        eh2 = left shooter, positive
        eh3 = left intake, negative
        
        ch5 = right plower servo
        eh5 = indexer servo, 0 closed, 0.5 open
        ehs0 = left plower servo
        */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftShooter.setDirection(DcMotor.Direction.FORWARD);
        rightShooter.setDirection(DcMotor.Direction.REVERSE);

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

        //behaviors 
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        
        leftIntake.setZeroPowerBehavior(BRAKE);
        rightIntake.setZeroPowerBehavior(BRAKE);

        //PID config, don't know if necessary 
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        // blockerPower = 0;//whatever number gets it to start upright
        // blockerServo.setPower(blockerPower);
        
        telemetry.addData("Status", "Initialized");
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit START
    @Override
    public void init_loop() {
    }

    //Code to run ONCE when the driver hits START
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        //intake is the butt 
        double forward = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if(gamepad1.x){
            imu.resetYaw();
        }
        
        fieldDrive(forward, strafe, rotate);
        intake();
        launch();
        plowerCheck();
    
        telemetry.addData("shooterL", leftShooter.getVelocity());
        telemetry.addData("shooterR", rightShooter.getVelocity());
        telemetry.addData("left Intake", leftIntake.getVelocity());
        telemetry.addData("right Intake", rightIntake.getVelocity());
        
        telemetry.addData("right pos", rightPlowerServo.getPosition());
        telemetry.addData("left pos", leftPlowerServo.getPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        /*
         * Send calculated power to wheels
         */
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    // keep mecanumDrive as this will call it after converting numbers
    // call this as main driving function
    void fieldDrive(double forward, double right, double rotate) {
        // convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // polar back to rectangular
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        mecanumDrive(newForward, newRight, rotate);
    }
    
    void intake(){
        if(gamepad1.left_trigger > 0.1){
            rightIntake.setVelocity(INDEXER_FEED_VELOCITY);
            leftIntake.setVelocity(INDEXER_FEED_VELOCITY);
        }else{
            rightIntake.setVelocity(0);
            leftIntake.setVelocity(0);
        }
    }
    
    void launch() {
        //spin to target if held and indexer on, coast if not and indexer off
        if (gamepad1.right_trigger > 0.1) {
            leftIntake.setVelocity(INDEXER_FEED_VELOCITY);
            rightIntake.setVelocity(INDEXER_FEED_VELOCITY);

            leftShooter.setVelocity(LAUNCHER_TARGET_VELOCITY);
            rightShooter.setVelocity(LAUNCHER_TARGET_VELOCITY);
            if (leftShooter.getVelocity() > LAUNCHER_MIN_VELOCITY && rightShooter.getVelocity() > LAUNCHER_MIN_VELOCITY){
                indexerServo.setPosition(0.5);
            }
        } else {
            leftShooter.setVelocity(LAUNCHER_COAST_VELOCITY);
            rightShooter.setVelocity(LAUNCHER_COAST_VELOCITY);
            indexerServo.setPosition(0);
        }
    }
        
        void plowerCheck(){
            if(gamepad1.left_bumper){
                leftPlowerServo.setPosition(0.48);
                rightPlowerServo.setPosition(0.52);
            }else{
                leftPlowerServo.setPosition(0.75);
                rightPlowerServo.setPosition(0.25);
            }
        }
    }
