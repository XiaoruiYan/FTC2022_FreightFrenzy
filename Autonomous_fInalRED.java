package FTC2021G2;

import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.Locale;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.Func;


@Autonomous(name = "AutoTestRED", group = "chad")

public class AutoTestRED extends LinearOpMode {
    
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    Servo servo;
    Servo servo1;
    Servo servo2;
    Servo servo3;
    DcMotor spin;
    DcMotor intake;
    DcMotorEx lift;
    ElapsedTime time = new ElapsedTime();
    
    //28 * 20 / (2ppi * 4.125)
    Double width = 16.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 20;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement
    Double conversion = cpi * bias;
    Boolean exit = false;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_DM.tflite";
    private static final String[] LABELS = {
      "Duck",
      "Marker"
    };

    private static final String VUFORIA_KEY =
            "AfI4Otz/////AAABmeHlvv7txUYnvzmaAi6+MCZtKfNcN+UnphkZqDpznHRhL0us4QFQwwO4Koqk9+92BrDzhwla0qnrOMqJEI839e4Z7suV9MOzEdxZ4/JjFoxPvzFMlJAugbxlssMmHLISxMNagyDOe3rrDM76Z9h0NaYXiL1uCfH6hgHVNefWGfBpeW1EJ6IXVyt/1h0YSNvIzNbw2+UyE4XuVMIh6nBc+zFXknDpo5GfA29G4wPo+wstrLyTxR1V+Z3qpeOyrfycweE/r4x4Tzgtsjri7IjpdhbmMaB4++DEsOoxVM89ZsWVGL6r6g0XVuGF8nTvuuSVmee8OGiOS6Xcn9eeuhT67RU+oNBa6Ps6Izy7AiPpFInk";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    
    private List<Recognition> recognitions;


    private boolean isDuckDetected = false;
    
    private float location = 0;
    private float locationXYZ = 0;
    
    public void runOpMode(){
        
         initVuforia();
         initTfod();
         
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); 
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        // composeTelemetry();

 
 
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.2, 10.0/5.0);
          //   tfod.setZoom(2.5, 16.0/9.0);
        }
        
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
         
        frontleft = hardwareMap.dcMotor.get("frontleft");
        frontright = hardwareMap.dcMotor.get("frontright");
        backleft = hardwareMap.dcMotor.get("backleft");
        backright = hardwareMap.dcMotor.get("backright");

        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        servo = hardwareMap.servo.get("servo");
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        servo3 = hardwareMap.servo.get("servo3");
        spin = hardwareMap.dcMotor.get("spin");
        intake = hardwareMap.dcMotor.get("intake");
        
        lift = hardwareMap.get(DcMotorEx.class,"lift");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        initServos();

        waitForStart();

            while (opModeIsActive()) {

                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      // step through the list of recognitions and display boundary info.
                      int i = 0;
                      for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                        
                        location = (recognition.getLeft() + recognition.getRight()) / 2f;
                      
                         // check label to see if the camera now sees a Duck         
                        if (recognition.getLabel().equals("Duck")) {           
                            isDuckDetected = true;                             
                            // telemetry.addData("Object Detected", "Duck");     
                        } 
                        
                        else {                                              
                            isDuckDetected = false;            
                            // telemetry.addData("Object Not Detected", "Duck");     
                        }

                        telemetry.update();
                    }
                }
                
                
                if (isDuckDetected == true) {
                    if (360f > location) {
                        telemetry.addData("duck location", 2);
                        locationXYZ = 2;
                    }
                    else {
                        telemetry.addData("duck location", 3);
                        locationXYZ = 3;
                    }
                }
                        
                if (isDuckDetected == false) {
                    telemetry.addData("duck location", 1);
                    locationXYZ = 1;
                }
                        
                telemetry.update();
    
            }
            
            //3
            if (locationXYZ == 3) {
                
                //Move to hub
                
                moveToPosition(-8, 0.3);
                
                sleep(300);
                
                strafeToPosition(-27.5, 0.55);
                
                sleep(300);
                
                moveToPosition(-16 ,0.6);
                
                lift(11, 1);
                servo10(0.55);
                sleep(600);
                servo1(0.5);
                sleep(300);
                servo2(1);
                sleep(600);
                
                servo1(0.8);
                sleep(500);
                
                servo10(0.1);
                sleep(450);
                
                lift(-11, 1);
                
                moveToPosition(5, 0.7);
                
                time.reset();
                frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while (time.milliseconds()< 800 && opModeIsActive()){
                    turnWithGyro(87, 0.7, 40);
                    telemetry.addData("gyro",-angles.firstAngle);
                    telemetry.update();
                }
                frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                moveToPosition(50, 0.7);
                
                
                time.reset();
                frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while (time.milliseconds()< 800 && opModeIsActive()){
                    turnWithGyro(19, 0.7, 40);
                    telemetry.addData("gyro",-angles.firstAngle);
                    telemetry.update();
                }
                frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
            
                moveToPosition(20, 0.2);
                moveToPosition(5, 0.1);
                
                spin(0.55);
                sleep(4000);
                
                moveToPosition(-10, 0.6);
                
                sleep(200);

                time.reset();
                frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while (time.milliseconds()< 800 && opModeIsActive()){
                    turnWithGyro(-85, 0.7, 40);
                    telemetry.addData("gyro",-angles.firstAngle);
                    telemetry.update();
                }
                frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                sleep(200);
                
                moveToPosition(57,1);
                strafeToPosition(48, 0.5);
                moveToPosition(49,1);
                
                // intake(-0.7);
                // sleep(1000);
                
 
                sleep(9999999);
            }
           
            //2 
            if (locationXYZ == 2) {
                
                moveToPosition(-8.5, 0.3);
                
                sleep(300);
                
                strafeToPosition(-27.5, 0.55);
                
                sleep(300);
                
        
                moveToPosition(-7, 0.7);
                
                servo1(0.9);
                sleep(300);

                servo10(0.65);
                sleep(600);
                servo1(0.57);
                sleep(900);
                
                moveToPosition(-5, 0.2);
                servo2(1);
                sleep(300);
                moveToPosition(5.5, 0.2);
                
                servo1(1);
                sleep(500);
                
                servo10(0.1);
                sleep(450);
                
                servo1(0.8);
                sleep(500);
                
                time.reset();
                frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while (time.milliseconds()<800 && opModeIsActive()){
                    turnWithGyro(88, 0.7, 40);
                    telemetry.addData("gyro",-angles.firstAngle);
                    telemetry.update();
                }
                frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                moveToPosition(51, 0.7);
                
                time.reset();
                frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while (time.milliseconds()< 800 && opModeIsActive()){
                    turnWithGyro(19, 0.7, 40);
                    telemetry.addData("gyro",-angles.firstAngle);
                    telemetry.update();
                }
                frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                moveToPosition(20, 0.2);
                moveToPosition(5, 0.1);
                
                spin(0.55);
                sleep(4000);
                
                moveToPosition(-10, 0.6);
                
                
                sleep(200);
                

                time.reset();
                frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while (time.milliseconds()< 800 && opModeIsActive()){
                    turnWithGyro(-85, 0.7, 40);
                    telemetry.addData("gyro",-angles.firstAngle);
                    telemetry.update();
                }
                frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                moveToPosition(57,1);
                strafeToPosition(48, 0.5);
                moveToPosition(49,1);
                
                // intake(-0.7);
                // sleep(1000);
                
                
                sleep(9999999);
                
            }
            
            //1
            if (locationXYZ == 1) {
                
                moveToPosition(-8, 0.3);
                
                strafeToPosition(-27, 0.7);
                
                sleep(300);
                
                moveToPosition(-4.2, 0.5);
                
                servo1(0.9);
                sleep(300);
                
                servo10(0.82);
                sleep(600);
                servo1(0.73);

                sleep(1000);
                moveToPosition(-7.1, 0.2);
                servo2(1);
                sleep(200);
                
                servo1(1);
                sleep(200);
                
                moveToPosition(8.9, 0.2);
                
                servo1(1);
                sleep(500);
                
                servo10(0.1);
                sleep(450);
                
                servo1(0.8);
                sleep(500);
                
                time.reset();
                frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while (time.milliseconds()<800 && opModeIsActive()){
                    turnWithGyro(87, 0.7, 40);
                    telemetry.addData("gyro",-angles.firstAngle);
                    telemetry.update();
                }
                frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                moveToPosition(51, 0.7);
                
                time.reset();
                frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while (time.milliseconds()< 800 && opModeIsActive()){
                    turnWithGyro(19, 0.7, 40);
                    telemetry.addData("gyro",-angles.firstAngle);
                    telemetry.update();
                }
                frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                moveToPosition(10, 0.2);
                moveToPosition(5, 0.1);
                
                spin(0.55);
                sleep(4000);
                
                moveToPosition(-10, 0.6);
                
                sleep(200);

                time.reset();
                frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while (time.milliseconds()< 800 && opModeIsActive()){
                    turnWithGyro(-85, 0.7, 40);
                    telemetry.addData("gyro",-angles.firstAngle);
                    telemetry.update();
                }
                frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                moveToPosition(57,1);
                strafeToPosition(48, 0.5);
                moveToPosition(49,1);
                
                // intake(-0.7);
                // sleep(1000);
                
                sleep(999999);
                
                
            }
    }
}    

    
    public void spin(double power) {
        spin.setPower(power);
    }
    
    public void intake(double power) {
        intake.setPower(power);
    }
    
    public void servo10(double posi) {
        servo.setPosition(posi);
        servo3.setPosition(1 - posi);
    }
    
    public void servo1(double posi) {
        servo1.setPosition(posi);
    }
    
    public void servo2(double posi) {
        servo2.setPosition(posi);
    }
    
    
    private void initVuforia() {
  
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
      tfodParameters.minResultConfidence = 0.8f;
      tfodParameters.isModelTensorFlow2 = true;
      tfodParameters.inputSize = 320;
      tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
      tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
    public void moveToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches * conversion));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
            if (exit){
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
                return;
            }
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;

    }
    
    public void turn(double inches, double speed){
        //
        //int move = (int)(Math.round(inches * conversion));
        int move = (int)(Math.round(inches * cpi * meccyBias));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() - move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
            if (exit){
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
                return;
            }
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }

    //
    public void turnWithGyro(double target, double speed, double cutOff){
        //<editor-fold desc="Initialize">
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        double difference;
        if (Math.abs(AngleUnit.normalizeDegrees(yaw-target))>cutOff){
            if (target > yaw){
                difference = -speed;
            } else {
                difference = speed;
            }
        } 
        
        else {
            difference = AngleUnit.normalizeDegrees(yaw-target)/cutOff*speed;
        }
        frontleft.setPower(-difference);
        backleft.setPower(-difference);
        frontright.setPower(difference);
        backright.setPower(difference);
        
        // frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
    }

    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.[]
     */
    public void strafeToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches * cpi * meccyBias));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){}
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }

    
    //
    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }

    //
    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input){
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontleft.setPower(input);
        backleft.setPower(input);
        frontright.setPower(-input);
        backright.setPower(-input);
    }
    
    public void lift(double inches, double speed){
        //
        int move = (int)(Math.round(inches * conversion));
        
        lift.setTargetPosition(lift.getCurrentPosition() + move);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(speed);

        while (lift.isBusy()){
            if (exit){
                lift.setPower(0);
    
                return;
            }
        }
        lift.setPower(0);
        return;

    }

    public void initServos() {
        servo1.setPosition(0.8);
        servo.setPosition(0.1);
        servo3.setPosition(0.9);
        servo2(0.7);
    }

}

