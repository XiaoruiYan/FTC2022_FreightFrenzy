package FTC2021G2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "ObjectDTCT")


public class ObjectDTCT extends LinearOpMode {

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
    
    
    @Override
    public void runOpMode() {
 
        initVuforia();
        initTfod();


        if (tfod != null) {
            tfod.activate();
            // tfod.setZoom(1, 10.0/5.0);
            tfod.setZoom(1.6, 16.0/9.0);
        }
    
                
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
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
                            telemetry.addData("Object Detected", "Duck");     
                        } 
                        else {                                              
                            isDuckDetected = false;            
                            telemetry.addData("Object Not Detected", "Duck");     
                        }
                        

                      telemetry.update();
                    }
                }

                // detect the location of the duck/obj
                if (isDuckDetected == true) {
                    if (360f < location) {
                        telemetry.addData("duck location", 3);
                    }
                    else {
                        telemetry.addData("duck location", 2);
                    }
                }
                        
                if (isDuckDetected == false) {
                    telemetry.addData("duck location", 1);
                }
                        
                telemetry.update();
    

            }
        }
    }



    private void initVuforia() {
  
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
      tfodParameters.minResultConfidence = 0.8f; //Lower the value for similar detections
      tfodParameters.isModelTensorFlow2 = true;
      tfodParameters.inputSize = 320;
      tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
      tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

    }
}
