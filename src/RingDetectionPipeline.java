package org.firstinspires.ftc.teamcode.mechanisms.camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RingDetectionPipeline extends OpenCvPipeline {
    //Tunable variables:
    public static final double surfaceToVolumeCost = 70.0;
    public static final double minimumArea = 1000.0;
    public static final int gaussBlurKernelSize = 5;

    public static final boolean useEasyOpenCVFrameFormat = true; //Set this to true when on the robot

    public static final boolean drawDetectionInformation = true;

    public volatile boolean detect = false;

    public volatile List<RingDetection> lastDetections;

    public Telemetry telemetry;

    public synchronized List<RingDetection> findRings(Mat in, byte sortOrder) {
        Mat hsv=new Mat();
        Imgproc.cvtColor(in,hsv,(useEasyOpenCVFrameFormat?Imgproc.COLOR_RGB2HSV:Imgproc.COLOR_BGR2HSV));

        Mat bin=new Mat();
        Core.inRange(hsv,calib.ringRangeA,calib.ringRangeB,bin);

        List<MatOfPoint> cs=new ArrayList<>();
        Mat h=new Mat();
        Imgproc.GaussianBlur(bin,bin,new Size(gaussBlurKernelSize,gaussBlurKernelSize),0);
        Imgproc.findContours(bin, cs, h, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        List<RingDetection> ret=new ArrayList<RingDetection>();
        for(MatOfPoint contour : cs) {
            Moments m=Imgproc.moments(contour);
            if(m.get_m00()<minimumArea) {continue;}
            Point center=new Point(m.get_m10() / m.get_m00(), m.get_m01() / m.get_m00());

            if(drawDetectionInformation) {
                List<MatOfPoint> contour_list=new ArrayList<>(1);
                contour_list.add(contour);
                Imgproc.drawContours(in,contour_list,-1,new Scalar(255,0,255),2,Imgproc.LINE_8);
                Imgproc.circle(in,center,20,new Scalar(255,0,255),2);
            }
            double lx=(center.x-calib.originX())*calib.pixel_cm();
            double ly=(center.y-calib.originY())*calib.pixel_cm();
            lx+=calib.reference_offset_x_cm();
            ly+=calib.reference_offset_y_cm();

            double r=Math.sqrt(Math.pow(lx,2.0)+Math.pow(ly,2.0));
            double theta=Math.toDegrees(Math.atan2(ly,lx));
            theta+=90; while(theta>180) {theta-=360;}
            double c=m.get_m00()+surfaceToVolumeCost*(m.get_m00()/Imgproc.arcLength(new MatOfPoint2f(contour.toArray()),true));

            ret.add(new RingDetection(r,theta,c));
            RingDetection.sortDetectionList(ret,sortOrder);
        }
        if(drawDetectionInformation) {
            Imgproc.cvtColor(in,hsv,(useEasyOpenCVFrameFormat?Imgproc.COLOR_RGB2HSV:Imgproc.COLOR_BGR2HSV));
            Imgproc.rectangle(hsv,new Point(0,0),new Point(in.size().width,in.size().height),new Scalar(calib.ringRangeA.val[0]+CameraCalibration.ringHueTolerance,255,255),10);
            Imgproc.cvtColor(hsv,in,(useEasyOpenCVFrameFormat?Imgproc.COLOR_HSV2RGB:Imgproc.COLOR_HSV2BGR));
        }
        return ret;
    }

    public void setTelemetry(Telemetry t) {
      this.telemetry = t;
    }

    public static CameraCalibration calib=new CameraCalibration();

  private int count=0;
    @Override
    public synchronized Mat processFrame(Mat input) {
      telemetry.addData("Process frame",""+count);
    count++;
        if (detect) {
            input=calib.getTopDown(input,input);
            lastDetections=findRings(input,RingDetection.BY_RADIUS);
        } else {
            calib.runPerspectiveCalibrationStep(input, telemetry);
        }
        return input;
    }
}