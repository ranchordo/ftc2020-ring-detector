package org.firstinspires.ftc.teamcode.mechanisms.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.mechanismhandlers.Mechanism;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class RingDetector extends Mechanism {
  public static final int size_x=640;
  public static final int size_y=480;

  RingDetectionPipeline pipeline;
  OpenCvCamera camera;
  @Override
  public void init(HardwareMap hwmap) {
    int cameraMonitorViewId = hwmap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwmap.appContext.getPackageName());
    camera = OpenCvCameraFactory.getInstance().createWebcam(hwmap.get(WebcamName.class,"Camera 1"),cameraMonitorViewId);
    pipeline = new RingDetectionPipeline();
    camera.setPipeline(pipeline);
    camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
      @Override
      public void onOpened() {}
    });
  }
  public void initTelemetry(Telemetry telemetry) {
    pipeline.setTelemetry(telemetry);
  }

  public void calibrate() {
    pipeline.telemetry.addLine("Calibrating camera...");
    camera.startStreaming(size_x,size_y,OpenCvCameraRotation.UPRIGHT);
    while (!RingDetectionPipeline.calib.isCalibrated()) {
      //Do nothing
    }
    camera.stopStreaming();
  }

  public void beginCalibration() {
    pipeline.telemetry.addLine("Calibrating camera...");

    camera.startStreaming(size_x,size_y,OpenCvCameraRotation.UPRIGHT);
  }

  public void terminateCalibration() {
    camera.stopStreaming();
  }

  public boolean isCalibrated() {
    return RingDetectionPipeline.calib.isCalibrated();
  }

  public List<RingDetection> detectAllRings(byte sortOrder) {
    pipeline.detect=true;
    pipeline.lastDetections=null;
    camera.startStreaming(size_x,size_y,OpenCvCameraRotation.UPRIGHT);
    while (pipeline.lastDetections == null) {
      //Do nothing
    }
    camera.stopStreaming();
    return pipeline.lastDetections;
  }
}