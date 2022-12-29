package FTC2021G2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;


@TeleOp(name="MecanumDual")

public class MecanumDual extends LinearOpMode {
        DcMotor frontleft;
        DcMotor backleft;
        DcMotor frontright;
        DcMotor backright;
        DcMotor TM;
        Servo servo;
        Servo servo3;
        Servo servo1;
        Servo servo2;
        Servo servo4;
        Servo servo5;
        DcMotor spin;
        DcMotor intake;
        DcMotorEx lift;
        TouchSensor touch;
        double RL = 0;
        double speedRatio;

    @Override
    public void runOpMode() {

        frontleft = hardwareMap.dcMotor.get("frontleft"); 
        backleft = hardwareMap.dcMotor.get("backleft");
        frontright = hardwareMap.dcMotor.get("frontright");
        backright = hardwareMap.dcMotor.get("backright");

        servo = hardwareMap.servo.get("servo");
        servo1 = hardwareMap.servo.get("servo1");
        spin = hardwareMap.dcMotor.get("spin");
        intake = hardwareMap.dcMotor.get("intake");
        lift = hardwareMap.get(DcMotorEx.class,"lift");
        TM = hardwareMap.dcMotor.get("TM");
        servo2 = hardwareMap.servo.get("servo2");
        servo3 = hardwareMap.servo.get("servo3");
        servo4 = hardwareMap.servo.get("servo4");
        servo5 = hardwareMap.servo.get("servo5");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        touch = hardwareMap.get(TouchSensor.class, "touch");
        
        
        initServos();

        double ly, rx, tr, denominator, frontLeftPower, backLeftPower, frontRightPower, backRightPower, speedRatio;

        NAILit20827();
        
        if (isStopRequested()) return;

        //Mecanum Base        
        while (opModeIsActive()) {
            
            if (gamepad1.right_bumper) {
                speedRatio = 0.3;
            }
            else {
                speedRatio = 1;
            }
        
            ly = gamepad1.left_stick_y;
            rx = -gamepad1.right_stick_x;
            tr = gamepad1.left_trigger - gamepad1.right_trigger;
            denominator = Math.max(Math.abs(ly) + Math.abs(rx) + Math.abs(tr), 1);

            frontLeftPower = (ly + rx + tr) / denominator;
            backLeftPower = (ly - rx + tr) / denominator;
            frontRightPower = (ly - rx - tr) / denominator;
            backRightPower = (ly + rx -  tr) / denominator;        
 
            frontleft.setPower(frontLeftPower * speedRatio);
            backleft.setPower(backLeftPower * speedRatio);
            frontright.setPower(frontRightPower * speedRatio);
            backright.setPower(backRightPower * speedRatio);
            
            
            if (gamepad1.y) {
                servo5.setPosition(0.53); 
            }
            
            else if (gamepad1.x) {
                servo5.setPosition(0); 
            }
            
            // Duck Spin
            if (gamepad2.right_bumper) {
                spin.setPower(0.68);
            }
            else if (gamepad2.left_bumper) {
                spin.setPower(1);
            }
            else {
                spin.setPower(0);
            }
            
            
            // Lift
            if (gamepad2.dpad_up) {
                lift.setPower(-0.65);
            }
            else if (gamepad2.dpad_down && touch.isPressed() == false) {
                lift.setPower(0.65);
            }
            else {
                lift.setPower(0);
            }
  
            // Arm
            if (gamepad2.a) {
                servo.setPosition(0.53); 
                servo3.setPosition(0.47);
            } 
            
            else if (gamepad2.x) {
                servo.setPosition(0.1); 
                servo3.setPosition(0.9);
                
            }
            
            // Small Servo
            if (gamepad2.y) {
                servo1.setPosition(0.8);
            }
            else if (gamepad2.b) {
                servo1.setPosition(0.5);
            }

            // Bucket Switch
            if (gamepad1.a) {
                servo2.setPosition(0.7);
            }
            else if (gamepad1.b) {
                servo2.setPosition(1);
            }
            
            // Claw Switch
            if (gamepad1.x) {
                servo4.setPosition(0);
                
            }
            else if (gamepad1.y) {
              servo4.setPosition(0.5);
                
            }
            
            // Intake
            if (gamepad2.right_trigger >= 0.1) {
                intake.setPower(-0.80);
            }
            else if (gamepad2.left_trigger >= 0.1) {
                intake.setPower(0.80);
            }
            else {
                intake.setPower(0);
            }

            
            telemetry.update();
    

            telemetry.addData("RL",RL);
            telemetry.addData("lift", lift.getCurrentPosition());

            telemetry.addData("gamepad2.right_stick_y", gamepad2.right_stick_y);
            telemetry.addData("FR", frontright.getCurrentPosition());
            telemetry.addData("FL", frontleft.getCurrentPosition());
            telemetry.addData("BR", backright.getCurrentPosition());
            telemetry.addData("BL", backleft.getCurrentPosition());

            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
    
    public void NAILit20827() {
        waitForStart();
    }
    
    public void initServos() {
        servo1.setPosition(0.8);
        servo.setPosition(0.1); 
        servo3.setPosition(0.9);

    }
}
