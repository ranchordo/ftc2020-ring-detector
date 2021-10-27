package main;

import java.util.Collections;
import java.util.List;

public class RingDetection implements Comparable<RingDetection> {
	public static final byte BY_RADIUS=0x00;
	public static final byte BY_CONFIDENCE=0x02;
	
	private static byte sortOrder;
	
	public static void sortDetectionList(List<RingDetection> lrd, byte sortOrder) {
		RingDetection.sortOrder=sortOrder;
		Collections.sort(lrd);
	}
	
	public double r; //Distance from calibrated reference point, centimeters
	public double theta; //Angle from straight ahead in degrees. To the right is positive, and to the left is negative. The transition point from -180 to 180 is in the back.
	public double confidence; //Higher is better
	
	public RingDetection(double nr, double nt, double c) {
		r=nr;
		theta=nt;
		confidence=c;
	}
	@Override
	public int compareTo(RingDetection o) {
		double q=(sortOrder==BY_RADIUS?r:0)+(sortOrder==BY_CONFIDENCE?confidence:0);
		double oq=(sortOrder==BY_RADIUS?o.r:0)+(sortOrder==BY_CONFIDENCE?o.confidence:0);
		return ((Double)q).compareTo((Double)oq);
	}
}
