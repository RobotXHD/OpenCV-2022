package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class OpenCV_test2 extends LinearOpMode {
    private OpenCvCamera webcam;
    private ContourPipeline pipeline;

    int varrez;
    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;

    private int minRectangleArea = 2000;
    private double leftBarcodeRangeBoundary = 0.3; //i.e 30% of the way across the frame from the left
    private double rightBarcodeRangeBoundary = 0.6; //i.e 60% of the way across the frame from the left

    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 150.0, 120.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new ContourPipeline(0.2, 0.2, 0.2, 0.2);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        //cred ca merge in loc de wait for start
        while(!isStopRequested()){
            //webcamul se porneste async (nu blocheza programul pana se porneste
            //deci trebuie asteptat cu detectia pana cand e pornit
            //ok
            //testeaza acum si sa veden cum merge si ce erori da
            //
            try{
                double rectangleArea = pipeline.getRectArea();

                //Print out the area of the rectangle that is found.
                telemetry.addData("Rectangle Area", rectangleArea);
                telemetry.addData("Position","X = " + pipeline.getRectX() + "    Y = " + pipeline.getRectY());

                //Check to see if the rectangle has a large enough area to be a marker.
                if (rectangleArea > minRectangleArea) {
                    //Then check the location of the rectangle to see which barcode it is in.
                    if (pipeline.getRectMidpointX() > rightBarcodeRangeBoundary * pipeline.getRectWidth()){
                        telemetry.addData("Barcode Position", "Right");
                        varrez = 3;
                    } else if (pipeline.getRectMidpointX() < leftBarcodeRangeBoundary * pipeline.getRectWidth()) {
                        telemetry.addData("Barcode Position", "Left");
                        varrez = 1;
                    } else {
                        telemetry.addData("Barcode Position", "Center");
                        varrez = 2;
                    }
                }
                telemetry.update();
            }catch (Exception e){
                telemetry.addData("E: ", e.getMessage());
                telemetry.update();
                //:)
            }
        }
    }
    /*
    public Thread Culoare = new Thread(new Runnable()
    {
        @Override
        public void run()
        {
            if(isStarted())
            {
                while (isStarted())
                {
                    if(pipeline.error){
                        telemetry.addData("Exception: ", pipeline.debug.getStackTrace());
                    }
                    // Only use this line of the code when you want to find the lower and upper values, using Ftc Dashboard (https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
                    // testing(pipeline);

                    // Watch our YouTube Tutorial for the better explanation

                    double rectangleArea = pipeline.getRectArea();

                    //Print out the area of the rectangle that is found.
                    telemetry.addData("Rectangle Area", rectangleArea);

                    //Check to see if the rectangle has a large enough area to be a marker.
                    if(rectangleArea > minRectangleArea){
                        //Then check the location of the rectangle to see which barcode it is in.
                        if(pipeline.getRectMidpointX() > rightBarcodeRangeBoundary * pipeline.getRectWidth()){
                            telemetry.addData("Barcode Position", "Right");
                            varrez=3;
                        }
                        else if(pipeline.getRectMidpointX() < leftBarcodeRangeBoundary * pipeline.getRectWidth()){
                            telemetry.addData("Barcode Position", "Left");
                            varrez=1;
                        }
                        else {
                            telemetry.addData("Barcode Position", "Center");
                            varrez=2;
                        }
                    }

                    telemetry.update();
                }
            }
        }
    });
     */
    public void testing(ContourPipeline pipeline){
        if(lowerRuntime + 0.05 < getRuntime()){
            crThreshLow += -gamepad1.left_stick_y;
            cbThreshLow += gamepad1.left_stick_x;
            lowerRuntime = getRuntime();
        }
        if(upperRuntime + 0.05 < getRuntime()){
            crThreshHigh += -gamepad1.right_stick_y;
            cbThreshHigh += gamepad1.right_stick_x;
            upperRuntime = getRuntime();
        }

        crThreshLow = inValues(crThreshLow, 0, 255);
        crThreshHigh = inValues(crThreshHigh, 0, 255);
        cbThreshLow = inValues(cbThreshLow, 0, 255);
        cbThreshHigh = inValues(cbThreshHigh, 0, 255);

        pipeline.configureScalarLower(0.0, crThreshLow, cbThreshLow);
        pipeline.configureScalarUpper(255.0, crThreshHigh, cbThreshHigh);

        telemetry.addData("lowerCr ", crThreshLow);
        telemetry.addData("lowerCb ", cbThreshLow);
        telemetry.addData("UpperCr ", crThreshHigh);
        telemetry.addData("UpperCb ", cbThreshHigh);
    }
    public double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
}
