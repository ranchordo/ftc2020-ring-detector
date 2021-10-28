package org.firstinspires.ftc.teamcode.mechanisms.camera;

import java.util.List;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class NestedContour {
    MatOfPoint c;
    List<MatOfPoint> cs;
    Point center;
    public NestedContour(MatOfPoint nc) {
        c=nc;
    }
    public static Double getContourArea(MatOfPoint c) {
        Moments m=Imgproc.moments(c);
        return m.get_m00();
    }
    public static Point getContourCenter(MatOfPoint c) {
        Moments m=Imgproc.moments(c);
        return new Point(m.get_m10() / m.get_m00(), m.get_m01() / m.get_m00());
    }
    public static double distanceFrom(Point a, Point b) {
      if(a==null) {
        throw new IllegalArgumentException("A is null.");
      }
      if(b==null) {
        throw new IllegalArgumentException("B is null");
      }
        return Math.sqrt(
                Math.pow(a.x-b.x,2)+
                Math.pow(a.y-b.y,2)
                );
    }
    public static Point farthestFrom(Point p, List<Point> c) {
        if(c.size()<1) {
            throw new IllegalArgumentException("Why are you empty, List<Point>?");
        }
        Point ret=null;
        double maxDist=0;
        for(Point a : c) {
            double d=distanceFrom(a,p);
            if(d>maxDist) {
                ret=a;
                maxDist=d;
            }
        }
        return ret;
    }
    public static Point closestTo(Point p, List<Point> c, double start) {
        if(c.size()<1) {
            throw new IllegalArgumentException("Why are you empty, List<Point>?");
        }
        Point ret=null;
        double minDist=start;
        for(Point a : c) {
            double d=distanceFrom(a,p);
            if(d<minDist) {
                ret=a;
                minDist=d;
            }
        }
        return ret;
    }
    public static Point farthestFrom(Point p, MatOfPoint c) {
        return farthestFrom(p,c.toList());
    }
    public static Point closestTo(Point p, MatOfPoint c, double start) {
        return closestTo(p, c.toList(), start);
    }
}