package FTC2021G2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="YXRmecanum")

public class YXRmecanum extends LinearOpMode {
        DcMotor motorFrontLeft;
        DcMotor motorBackLeft;
        DcMotor motorFrontRight;
        DcMotor motorBackRight;


    @Override
    public void runOpMode() {

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

       
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // lift.setTargetPosition(lift.getCurrentPosition());


        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        double ly, rx, lx, denominator, frontLeftPower, backLeftPower, frontRightPower, backRightPower;

        waitForStart();
        
        if (isStopRequested()) return;

        //Mecanum Base        
        while (opModeIsActive()) {
            
            ly = -gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;
            lx = gamepad1.left_stick_x;

            denominator = Math.max(Math.abs(ly) + Math.abs(rx) + Math.abs(lx), 1);
            frontLeftPower = (ly + rx + lx) / denominator;
            backLeftPower = (ly - rx + lx) / denominator;
            frontRightPower = (ly - rx - lx) / denominator;
            backRightPower = (ly + rx -  lx) / denominator;

         
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);


           
        }
    }
}


