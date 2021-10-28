package org.firstinspires.ftc.teamcode.mechanisms.camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class CameraCalibration {
	public static final float qr_side_length_cm=18.95f;

	private volatile float reference_offset_x_cm=0;
	private volatile float reference_offset_y_cm=0;

	//Tunable variables, part 2:
	public static final Scalar satRangeA=new Scalar(0,190,150);
	public static final Scalar satRangeB=new Scalar(255,300,300);

	public static final int satGaussBlurKernelSize=5;

	public static final int relativeMinHueFieldSize=100000;

	public static final int ringHueTolerance=30;

	public static final int MIN_STEPS=20;
	//End

	private static final int size=50;
	private static final int offX=400;
	private static final int offY=400+size/2;
	private static final MatOfPoint2f default_dst_coords=new MatOfPoint2f(
			new Point(offX, offY),
            new Point(offX+size-1,offY),
            new Point(offX,offY+size-1),
            new Point(offX+size-1,offY+size-1));
	private volatile Point[] points=new Point[4];
	private volatile Point[] points_sum=new Point[4];
	private volatile Point[] points_avg=new Point[4];
	private volatile float hue_sum=0;
	private volatile int hueDetectionCount=0;
	private volatile int detectionCount=0;

	public volatile Scalar ringRangeA;
	public volatile Scalar ringRangeB;

	private volatile double pixel_cm=0;

	private volatile double originX;
	private volatile double originY;

	public double originX() {
		cc();
		return originX;
	}
	public double originY() {
		cc();
		return originY;
	}
	public double pixel_cm() {
		cc();
		return pixel_cm;
	}

	private MatOfPoint2f dst_coords=null;
	public synchronized boolean isCalibrated() {
		return (detectionCount>=MIN_STEPS)&&(hueDetectionCount>=MIN_STEPS);
	}
	public synchronized int getDetectionCount() {
		return detectionCount;
	}

	public synchronized boolean runPerspectiveCalibrationStep(Mat in, Telemetry telemetry) {
		if(isCalibrated()) {
			return true;
		}
		if(detectionCount<MIN_STEPS) {
			float scaleX=in.width()/640.0f;
			float scaleY=in.height()/480.0f;
			MatOfPoint2f help_coords=new MatOfPoint2f(
					new Point(178.0*scaleX,156.0*scaleY),
					new Point(402.6*scaleX,157.0*scaleY),
					new Point(131.0*scaleX,357.0*scaleY),
					new Point(429.4*scaleX,356.0*scaleY)
			);
			int sizeH=170;
			int offXH=250;
			int offYH=250;
			MatOfPoint2f help_dst_coords=new MatOfPoint2f(
					new Point((offXH)*scaleX, (offYH)*scaleY),
		            new Point((offXH+sizeH-1)*scaleX,(offYH)*scaleY),
		            new Point((offXH)*scaleX,(offYH+sizeH-1)*scaleY),
		            new Point((offXH+sizeH-1)*scaleX,(offYH+sizeH-1)*scaleY));

			Mat warpMatHelper=Imgproc.getPerspectiveTransform(help_coords,help_dst_coords);
			Mat warpMatHelperInv=warpMatHelper.inv();
			Mat in_helped=new Mat();
			Imgproc.warpPerspective(in, in_helped, warpMatHelper, in.size());
		// 	Imgproc.warpPerspective(in, in, warpMatHelper, in.size());

			List<Point> c=detectCalibrationCode(in_helped,telemetry);
			if(c!=null) {

			  if(c.size()!=4) {
			    throw new IllegalStateException("Whaaaat?");
			  }
				detectionCount++;
				if(points[0]==null) {for(int i=0;i<4;i++) {points[i]=new Point();}}
				if(points_sum[0]==null) {for(int i=0;i<4;i++) {points_sum[i]=new Point(0,0);}}

        double start=Math.sqrt(
          Math.pow(in.width(),2.0)+
          Math.pow(in.height(),2.0)
          );
				points[0].set(new MatOfPoint(NestedContour.closestTo(new Point(0,0),c,start)).get(0,0));
				points[1].set(new MatOfPoint(NestedContour.closestTo(new Point(in.width(),0),c,start)).get(0,0));
				points[2].set(new MatOfPoint(NestedContour.closestTo(new Point(0,in.height()),c,start)).get(0,0));
				points[3].set(new MatOfPoint(NestedContour.closestTo(new Point(in.width(),in.height()),c,start)).get(0,0));


				telemetry.addData("Got QR detection",String.valueOf(detectionCount));

				MatOfPoint2f l=new MatOfPoint2f();
				Core.perspectiveTransform(new MatOfPoint2f(points), l, warpMatHelperInv);

				for(int i=0;i<4;i++) {points[i].set(l.get(i,0));}
        if(RingDetectionPipeline.drawDetectionInformation) {
				  Imgproc.circle(in,points[0],20,new Scalar(255,0,255),2);
				  Imgproc.circle(in,points[1],20,new Scalar(255,0,255),2);
				  Imgproc.circle(in,points[2],20,new Scalar(255,0,255),2);
				  Imgproc.circle(in,points[3],20,new Scalar(255,0,255),2);
        }

				for(int i=0;i<4;i++) {points_sum[i]=new Point(points_sum[i].x+points[i].x,points_sum[i].y+points[i].y);}

			}
		}
		if(detectionCount==MIN_STEPS) {
			recalculatePersp(in.size(),telemetry);
			telemetry.addLine("Camera perspective calibrated.");
			detectionCount++;
		}
		if(detectionCount>=MIN_STEPS) {
			Mat td=new Mat();
			getTopDown(in,td);
			Rect r=new Rect((int)dst_coords.get(0,0)[0],(int)dst_coords.get(0,0)[1],(int)(dst_coords.get(1,0)[0]-dst_coords.get(0,0)[0]),(int)(dst_coords.get(2,0)[1]-dst_coords.get(0,0)[1]));
			td=new Mat(td,r);
			calibrateRingColor(td, telemetry);
		}
		if(isCalibrated()) {
			ringRangeA=new Scalar(hue_sum/hueDetectionCount-ringHueTolerance,satRangeA.val[1],satRangeA.val[2]);
			ringRangeB=new Scalar(hue_sum/hueDetectionCount+ringHueTolerance,satRangeB.val[1],satRangeB.val[2]);
			telemetry.addLine("Successful overall calibration.");
		}
		return false;
	}
	private void computePointsAvg() {
		for(int i=0;i<4;i++) {
			points_avg[i]=new Point(
					points_sum[i].x/(float)detectionCount,
					points_sum[i].y/(float)detectionCount);
		}
	}
	private volatile Mat persp;
	public synchronized Mat getPersp(Size s) {
		if(detectionCount<MIN_STEPS) {
			throw new IllegalStateException("Camera perspective is not calibrated yet.");
		}
		return persp;
	}
	private synchronized void cc() {
		if(!isCalibrated()) {
			throw new IllegalStateException("Camera is not calibrated yet. Please run at least "+MIN_STEPS+" successful perspective calibration steps and retry.");
		}
	}
	private synchronized void calibrateRingColor(Mat in, Telemetry telemetry) {
		Mat hsv=new Mat();
		Imgproc.cvtColor(in,hsv,(RingDetectionPipeline.useEasyOpenCVFrameFormat?Imgproc.COLOR_RGB2HSV:Imgproc.COLOR_BGR2HSV));
		Mat bin=new Mat();
		Core.inRange(hsv,satRangeA,satRangeB,bin);
		double r=relativeMinHueFieldSize*(in.size().area()/(640.0*480.0));
		if((Core.sumElems(bin).val)[0]>r) {
			telemetry.addData("Ring hue detection",""+(hueDetectionCount+1));
			Scalar m=Core.mean(hsv,bin);
			hue_sum+=(float)m.val[0];
			hueDetectionCount+=1;
		}
	}

	public synchronized void recalculatePersp(Size s, Telemetry telemetry) {
		computePointsAvg();
		persp=Imgproc.getPerspectiveTransform(new MatOfPoint2f(points_avg),default_dst_coords);
		MatOfPoint2f mop=new MatOfPoint2f(
				new Point(0,0),
				new Point(0,s.height),
				new Point(s.width,s.height),
				new Point(s.width,0));
		Core.perspectiveTransform(mop, mop, persp);

		double minX=Math.min(
				mop.get(0,0)[0],
				Math.min(
						mop.get(1,0)[0],
						Math.min(
								mop.get(2,0)[0],
								mop.get(3,0)[0])));
		double minY=Math.min(
				mop.get(0,0)[1],
				Math.min(
						mop.get(1,0)[1],
						Math.min(
								mop.get(2,0)[1],
								mop.get(3,0)[1])));
		double maxX=Math.max(
				mop.get(0,0)[0],
				Math.max(
						mop.get(1,0)[0],
						Math.max(
								mop.get(2,0)[0],
								mop.get(3,0)[0])));
		double maxY=Math.max(
				mop.get(0,0)[1],
				Math.max(
						mop.get(1,0)[1],
						Math.max(
								mop.get(2,0)[1],
								mop.get(3,0)[1])));
		double scaleX=s.width/(maxX-minX);
		double scaleY=s.height/(maxY-minY);
		double scale=Math.min(scaleX,scaleY);

		Point[] dst_coords_arr=new Point[default_dst_coords.height()];
		for(int i=0;i<default_dst_coords.height();i++) {
			dst_coords_arr[i]=new Point((default_dst_coords.get(i,0)[0]*scale)-(minX*scale),(default_dst_coords.get(i,0)[1]*scale)-(minY*scale));
		}
		dst_coords=new MatOfPoint2f(dst_coords_arr);
		persp=Imgproc.getPerspectiveTransform(new MatOfPoint2f(points_avg),dst_coords);

		double dist_pix=Math.sqrt(Math.pow(dst_coords.get(3,0)[0]-dst_coords.get(2,0)[0],2.0)+Math.pow(dst_coords.get(3,0)[1]-dst_coords.get(2,0)[1],2.0));
		pixel_cm=qr_side_length_cm/dist_pix;
		telemetry.addData("IRL distance for one pixel (cm)",""+pixel_cm);
		originX=dst_coords.get(2,0)[0];
		originY=dst_coords.get(2,0)[1];
	}
	public static List<Point> detectCalibrationCode(Mat in, Telemetry telemetry) {
		Mat gray = new Mat();
		Imgproc.cvtColor(in, gray, (RingDetectionPipeline.useEasyOpenCVFrameFormat?Imgproc.COLOR_RGB2GRAY:Imgproc.COLOR_BGR2GRAY));
		Imgproc.GaussianBlur(gray, gray, new Size(5, 5), 0);
		Mat edges = new Mat();
		Imgproc.Canny(gray, edges, 30, 200);
		Mat h = new Mat();
		List<MatOfPoint> cs = new ArrayList<>();
		Imgproc.findContours(edges, cs, h, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
		List<MatOfPoint> scs = new ArrayList<>();
		for (MatOfPoint c : cs) {
			double p = Imgproc.arcLength(new MatOfPoint2f(c.toArray()), true);
			MatOfPoint2f oc = new MatOfPoint2f();
			Imgproc.approxPolyDP(new MatOfPoint2f(c.toArray()), oc, 0.03 * p, true);
			if (oc.height() == 4) {
				scs.add(new MatOfPoint(oc.toArray()));
			}
		}
		List<NestedContour> ncs=scs.stream().map(m->(new NestedContour(m))).collect(Collectors.toList());
		for(NestedContour n : ncs) {
			Moments m=Imgproc.moments(n.c);
			n.center=new Point(m.get_m10() / m.get_m00(), m.get_m01() / m.get_m00());
		}
		for(NestedContour nc : ncs) {
			nc.cs=new ArrayList<>();
			MatOfPoint2f c2f=new MatOfPoint2f(nc.c.toArray());
			for(NestedContour n : ncs) {
				if(n==nc) {continue;}
				if(Imgproc.pointPolygonTest(c2f,n.center,false)>0.0) {
					nc.cs.add(n.c);
				}
			}
		}
		List<NestedContour> fcs=new ArrayList<>();
		for(NestedContour n : ncs) {
			if(n.cs.size()>=3) {
				fcs.add(n);
			}
		}
		List<NestedContour> fs=new ArrayList<>();
		for(NestedContour n : fcs) {
			boolean unique=true;
			for(NestedContour f : fs) {
				for(MatOfPoint c : f.cs) {
					if(n.c==c) {
						unique=false;
						break;
					}
				}
				if(!unique) {
					break;
				}
			}
			if(unique) {
				fs.add(n);
			}
			n.cs.add(n.c);
			Collections.sort(n.cs,(o1, o2) -> NestedContour.getContourArea(o1).compareTo(NestedContour.getContourArea(o2)));
			n.c=n.cs.get(n.cs.size()-1);
		}
		if(fs.size()!=4) {
		  return null;
		}
		List<MatOfPoint> bigContours=fs.stream().map(m->m.c).collect(Collectors.toList());
		List<Point> centers=bigContours.stream().map(m->NestedContour.getContourCenter(m)).collect(Collectors.toList());
		Point center=new Point(0,0);
		centers.forEach(p->{center.x+=p.x; center.y+=p.y;});
		center.x/=fs.size();
		center.y/=fs.size();
		List<Point> corners=bigContours.stream().map(m->NestedContour.farthestFrom(center,m)).collect(Collectors.toList());
		if(118==254) {
			Imgproc.drawContours(in, bigContours, -1, new Scalar(255, 0, 255), 2, Imgproc.LINE_8);
			Imgproc.circle(in, center, 5, new Scalar(0, 255, 0), 2);
			for (Point p : corners) {
				Imgproc.circle(in, p, 15, new Scalar(255, 255, 0), 2);
			}
		}
		return corners;
	}
	public synchronized Mat getTopDown(Mat in, Mat ret) {
		Imgproc.warpPerspective(in,ret,getPersp(in.size()),in.size());
		return ret;
	}
	public float reference_offset_x_cm() {
		return reference_offset_x_cm;
	}
	public void setReference_offset_x_cm(float reference_offset_x_cm) {
		this.reference_offset_x_cm = reference_offset_x_cm;
	}
	public float reference_offset_y_cm() {
		return reference_offset_y_cm;
	}
	public void setReference_offset_y_cm(float reference_offset_y_cm) {
		this.reference_offset_y_cm = reference_offset_y_cm;
	}
}