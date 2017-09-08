/* Class modified from OpenCV Sample*/
package abr.predator;

import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class ColorBlobDetector {
    // Lower and Upper bounds for range checking in HSV color space
    private Scalar mLowerBound = new Scalar(0);
    private Scalar mUpperBound = new Scalar(0);
    // Minimum contour area in percent for contours filtering
    private static double mMinContourArea = 0.1;
    // Color radius for range checking in HSV color space
    private Scalar mColorRadius = new Scalar(25,50,50,0);
    private Mat mSpectrum = new Mat();
    private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();
    private float[] radius;
    private Point center;
    private double screenCenterX, screenCenterY;
    private Point origin;
    private Point moment;
    private double centerDifference;

    // Cache
    Mat mPyrDownMat = new Mat();
    Mat mHsvMat = new Mat();
    Mat mMask = new Mat();
    Mat mDilatedMask = new Mat();
    Mat mHierarchy = new Mat();

    public void setColorRadius(Scalar radius) {
        mColorRadius = radius;
    }

    public void setHsvColor(Scalar hsvColor) {
        double minH = (hsvColor.val[0] >= mColorRadius.val[0]) ? hsvColor.val[0]-mColorRadius.val[0] : 0;
        double maxH = (hsvColor.val[0]+mColorRadius.val[0] <= 255) ? hsvColor.val[0]+mColorRadius.val[0] : 255;

        mLowerBound.val[0] = minH;
        mUpperBound.val[0] = maxH;

        mLowerBound.val[1] = hsvColor.val[1] - mColorRadius.val[1];
        mUpperBound.val[1] = hsvColor.val[1] + mColorRadius.val[1];

        mLowerBound.val[2] = hsvColor.val[2] - mColorRadius.val[2];
        mUpperBound.val[2] = hsvColor.val[2] + mColorRadius.val[2];

        mLowerBound.val[3] = 0;
        mUpperBound.val[3] = 255;

        Mat spectrumHsv = new Mat(1, (int)(maxH-minH), CvType.CV_8UC3);

        for (int j = 0; j < maxH-minH; j++) {
            byte[] tmp = {(byte)(minH+j), (byte)255, (byte)255};
            spectrumHsv.put(0, j, tmp);
        }

        Imgproc.cvtColor(spectrumHsv, mSpectrum, Imgproc.COLOR_HSV2RGB_FULL, 4);
    }

    public Mat getSpectrum() {
        return mSpectrum;
    }

    public void setMinContourArea(double area) {
        mMinContourArea = area;
    }

    public void process(Mat rgbaImage) {
        origin = new Point(rgbaImage.cols()/2+30,rgbaImage.rows()/2);
        Imgproc.pyrDown(rgbaImage, mPyrDownMat);
        Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);

        Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

        Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
        Imgproc.dilate(mMask, mDilatedMask, new Mat());

        screenCenterX = (double) mDilatedMask.cols()/2;
        screenCenterY = (double) mDilatedMask.rows()/2;

        Log.i("yellow screen",String.format("%.2f",screenCenterX));

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

        //Imgproc.findContours(mDilatedMask, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(mDilatedMask, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find max contour area
        double maxArea = 0;
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > maxArea)
                maxArea = area;
        }

        // Filter contours by area and resize to fit the original image size
/*        mContours.clear();
        each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint contour = each.next();
            if (Imgproc.contourArea(contour) > mMinContourArea*maxArea) {
                Core.multiply(contour, new Scalar(4,4), contour);
                mContours.add(contour);
            }
        }*/

        mContours.clear();
        each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint contour = each.next();
            if (Imgproc.contourArea(contour) == maxArea) {
                Core.multiply(contour, new Scalar(4, 4), contour);
                mContours.add(contour);
            }
        }
        if (mContours.size() > 0){
            double area = Imgproc.contourArea(mContours.get(0));
            moment = calculateMoment(mContours.get(0));
            radius = new float[1];
            center = new Point();
            centerDifference = calculateCenterDifference(center,screenCenterX);
            Imgproc.minEnclosingCircle(new MatOfPoint2f(mContours.get(0).toArray()), center, radius);
            //mBalls.add(new circle(center.x, center.y, calib.getBallRadius()));
        } else {
            double area = 0.0;
            radius = new float[1];
            radius[0] = 0.0f;
            center = new Point();
            moment = new Point();
            centerDifference = 0.0;
        }

    }

    //Added - calculates center of mass of blob with largest area
    public Point calculateMoment(MatOfPoint contour){
        List<Point> contourPoints = contour.toList();
        double momentX = 0;
        double momentY = 0;
        for(int i = 0; i < contourPoints.size(); i++){
            momentX += contourPoints.get(i).x;
            momentY += contourPoints.get(i).y;
        }
        momentX = momentX/contourPoints.size();
        momentY = momentY/contourPoints.size();

        momentX = momentX - screenCenterX;
        momentY = momentY - screenCenterY;

        return new Point(momentX,momentY);
    }

    private double calculateCenterDifference(Point center, double screenCenterX) {
        return (center.x - screenCenterX);
    }

    public List<MatOfPoint> getContours() {
        return mContours;
    }

    public float[] getRadius() {
        return radius;
    }

    public Point getCenter() {
        return center;
    }

    public Point getOrigin() { return origin; }

    public void setOrigin(Point orig) { origin = orig; }

    public Point getMoment() { return moment; }

    public double getCenterDifference() { return  centerDifference; }
}
