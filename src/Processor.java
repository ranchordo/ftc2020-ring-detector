package main;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class Processor {
//	//Tunable variables:
//	public static final double surfaceToVolumeCost=70.0;
//	public static final double minimumArea=1000.0;
//	public static final int gaussBlurKernelSize=5;
//	
//	public static final boolean useEasyOpenCVFrameFormat=false; //Set this to true when on the robot
//	
//	public static final boolean drawDetectionInformation=true;
//	
//	public static List<RingDetection> findRings(Mat in, byte sortOrder) {
//		Mat hsv=new Mat();
//		Imgproc.cvtColor(in,hsv,(useEasyOpenCVFrameFormat?Imgproc.COLOR_RGB2HSV:Imgproc.COLOR_BGR2HSV));
//		
//		Mat bin=new Mat();
//		Core.inRange(hsv,calib.ringRangeA,calib.ringRangeB,bin);
//		
//		List<MatOfPoint> cs=new ArrayList<>();
//		Mat h=new Mat();
//		Imgproc.GaussianBlur(bin,bin,new Size(gaussBlurKernelSize,gaussBlurKernelSize),0);
//		Imgproc.findContours(bin, cs, h, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//		
//		List<RingDetection> ret=new ArrayList<RingDetection>();
//		for(MatOfPoint contour : cs) {
//			Moments m=Imgproc.moments(contour);
//			if(m.get_m00()<minimumArea) {continue;}
//			Point center=new Point(m.get_m10() / m.get_m00(), m.get_m01() / m.get_m00());
//			
//			if(drawDetectionInformation) {
//				List<MatOfPoint> contour_list=new ArrayList<>(1);
//				contour_list.add(contour);
//				Imgproc.drawContours(in,contour_list,-1,new Scalar(255,0,255),2,Imgproc.LINE_8);
//				Imgproc.circle(in,center,20,new Scalar(255,0,255),2);
//			}
//			double lx=(center.x-calib.originX())*calib.pixel_cm();
//			double ly=(center.y-calib.originY())*calib.pixel_cm();
//			lx+=calib.reference_offset_x_cm();
//			ly+=calib.reference_offset_y_cm();
//			
//			double r=Math.sqrt(Math.pow(lx,2.0)+Math.pow(ly,2.0));
//			double theta=Math.toDegrees(Math.atan2(ly,lx));
//			theta+=90; while(theta>180) {theta-=360;}
//			double c=m.get_m00()+surfaceToVolumeCost*(m.get_m00()/Imgproc.arcLength(new MatOfPoint2f(contour.toArray()),true));
//			
//			ret.add(new RingDetection(r,theta,c));
//			RingDetection.sortDetectionList(ret,sortOrder);
//		}
//		if(drawDetectionInformation) {
//			Imgproc.cvtColor(in,hsv,(useEasyOpenCVFrameFormat?Imgproc.COLOR_RGB2HSV:Imgproc.COLOR_BGR2HSV));
//			Imgproc.rectangle(hsv,new Point(0,0),new Point(in.size().width,in.size().height),new Scalar(calib.ringRangeA.val[0]+CameraCalibration.ringHueTolerance,255,255),10);
//			Imgproc.cvtColor(hsv,in,(useEasyOpenCVFrameFormat?Imgproc.COLOR_HSV2RGB:Imgproc.COLOR_HSV2BGR));
//		}
//		return ret;
//	}
//	
//	static CameraCalibration calib=new CameraCalibration();
	public static Mat process(Mat in) {
//		if(calib.isCalibrated()) {
//			input=calib.getTopDown(input,input);
//			findRings(input,RingDetection.BY_RADIUS);
//		} else {
//			calib.runPerspectiveCalibrationStep(input);
//		}
		
		Mat gray=new Mat();
		Imgproc.cvtColor(in,gray,Imgproc.COLOR_BGR2GRAY);
		Imgproc.GaussianBlur(gray,gray,new Size(5,5),0);
		Mat edges=new Mat();
		Imgproc.Canny(gray,edges,30,200);
		Mat h=new Mat();
		List<MatOfPoint> cs=new ArrayList<>();
		Imgproc.findContours(edges, cs, h, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
		List<MatOfPoint> scs=new ArrayList<>();
		for(MatOfPoint c : cs) {
			double p=Imgproc.arcLength(new MatOfPoint2f(c.toArray()),true);
			MatOfPoint2f oc=new MatOfPoint2f();
			Imgproc.approxPolyDP(new MatOfPoint2f(c.toArray()),oc,0.03*p,true);
			if(oc.height()==4) {
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
		if(fs.size()==4) {
			List<MatOfPoint> bigContours=fs.stream().map(m->m.c).collect(Collectors.toList());
			List<Point> centers=bigContours.stream().map(m->NestedContour.getContourCenter(m)).collect(Collectors.toList());
			Point center=new Point(0,0);
			centers.forEach(p->{center.x+=p.x; center.y+=p.y;});
			center.x/=fs.size();
			center.y/=fs.size();
			List<Point> corners=bigContours.stream().map(m->NestedContour.farthestFrom(center,m)).collect(Collectors.toList());
			Imgproc.drawContours(in,bigContours,-1,new Scalar(255,0,255),2,Imgproc.LINE_8);
			Imgproc.circle(in,center,5,new Scalar(0,255,0),2);
			for(Point p : corners) {
				Imgproc.circle(in,p,15,new Scalar(255,0,0),2);
			}
		}
//		
//		Imgproc.cvtColor(gray,in,Imgproc.COLOR_GRAY2BGR);
		return in;
	}
}
