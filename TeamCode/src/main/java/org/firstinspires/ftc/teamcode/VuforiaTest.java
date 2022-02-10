package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
@Autonomous
public class VuforiaTest extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY = "ARfI6Rf/////AAABmU/mcRFizUk8vW6MJ0HoM/AssoeQGsrqTT5ogCM+vcqjTXlD/GTcRqmOyBVBQVVFcUrdhjjbf+PU9i33q2ilWheaahM/uEwElUobHmK8kysK95NrsIKgZCEES/3zS6fZO5taMrEU1IsHZyXvO6XMQy9SjSFjEUVPXdtwk01l7yQ+uWFrCDyBU3OGz9pYC6eCR2m5q8D236g6yHqsxxraBjy32DsZDX7QRNm/9SNMLMrTkdIwn4jr4uT83hrNnpHV59DT5jDb1n3pnlSaHSyPwrM7kbZLWTdmjr+T50OeoP9vjArcCkd/Ogvrmm77QHXD/GEpvyKjX9Ejw3eTKBM0HOjmj73r8q/aT0x2KAIuKguI";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    String rezultat = "None";
    String rezultat1 = "None";
    int varrez=0;

    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0/9.0);
        }
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();
        while(isStarted()){
            Detectare.start();
            if(rezultat == "None" && rezultat1 == "None") varrez=1;
            else if(rezultat == "Duck" && rezultat1 == "Marker")  varrez=2;
            else if(rezultat == "Marker" && rezultat1 == "Duck")  varrez=3;
            telemetry.addData("var",varrez );
            telemetry.update();
        }
    }
    public Thread Detectare = new Thread(new Runnable() {
        @Override
        public void run() {
            if (isStarted()) {
                while (isStarted()) {
                    if(tfod != null){
                        tfod.shutdown();
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
                                rezultat = recognition.getLabel();
                            }
                            telemetry.update();
                        }
                        if (updatedRecognitions.size()==0)
                            rezultat = "None";
                        telemetry.addData("var",rezultat );
                    }
                }
            }
        }
    });
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
}
