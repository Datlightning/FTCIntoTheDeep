package org.firstinspires.ftc.teamcode.vision;

//import com.acmerobotics.dashboard.config.Config;

//import com.acmerobotics.dashboard.config.Config;

//import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;


//@Config//disable this for thing
public class TheBigBrainAlgorithm extends OpenCvPipeline {
    private double distance(Point p1, Point p2) {
        return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
    }
    public class DetectedSample{
        public RotatedRect rect;
        public double area, lateral, linear, score, separation = 0;


        public String type;
        public DetectedSample(String type, RotatedRect rect, double area, double lateral, double linear){
            this.rect = rect;
            this.area = area;
            this.lateral = lateral;
            this.linear = linear;
            this.score = lateral + 50 * Math.abs(linear) + Math.abs(area - approx_sample_size);
            ;
            this.type = type;
        }
        public double getScore(){
            return score;
        }

    }
    public class SampleComparator implements Comparator<DetectedSample> {
        @Override
        public int compare(DetectedSample r1, DetectedSample r2) {
            // Sort by area in descending order (max-heap)
            return Double.compare(r1.getScore(), r2.getScore());
        }
    }
    public boolean debug = false;
    public class DetectedSampleList{
        PriorityQueue<DetectedSample> queue;
        PriorityQueue<DetectedSample> valid_options;
        public DetectedSampleList(){
            queue = new PriorityQueue<>(new SampleComparator());
            valid_options = new PriorityQueue<>(new SampleComparator());

        }
        public void addSample(DetectedSample sample){
            queue.add(sample);
            if(sample.type.equals("Yellow")){
                valid_options.add(sample);
            }
        }
        public boolean intersectsSample(DetectedSample sample1, DetectedSample sample2){
            Point[] points1 = new Point[4];
            Point[] points2 = new Point[4];
            sample1.rect.points(points1);
            sample2.rect.points(points2);
            Point[] long_edge1_1 = new Point[0];
            Point[] long_edge1_2 = new Point[0];
            Point p1, p2;
            double rect1_long_distance = 0;
            for (int i = 0; i < 4; i++){
                p1 = points1[i];
                p2 = points1[(i + 1) % 4];
                double temp1 = distance(p1,p2);
                if (temp1 > rect1_long_distance){
                    rect1_long_distance = temp1;
                    long_edge1_1 = new Point[]{p1, p2};
                    long_edge1_2 = new Point[]{points1[(i + 2) % 4], points1[(i + 3) % 4]};
                }

            }
            Point[] center_line = new Point[]{sample1.rect.center, sample2.rect.center};

            return isIntersectionValid(long_edge1_1, center_line) || isIntersectionValid(long_edge1_2, center_line);

        }
        public boolean isIntersectionValid(Point[] points1, Point[] points2) {
            // Line segment 1: p1 -> p2
            // Line segment 2: p3 -> p4
            Point p1 = points1[0];
            Point p2 = points1[1];
            Point p3 = points2[0];
            Point p4 = points2[1];

            double denominator = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);

            // If denominator is zero, the lines are parallel or coincident
            if (denominator == 0) {
                return false; // No intersection
            }

            // Calculate the intersection point using the parameterized formula
            double t = ((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)) / denominator;
            double u = ((p1.x - p3.x) * (p1.y - p2.y) - (p1.y - p3.y) * (p1.x - p2.x)) / denominator;

            // Check if the intersection point lies within both line segments (t and u must be between 0 and 1)
            // Calculate the intersection point
            // No intersection within the line segments
            return t >= 0 && t <= 1 && u >= 0 && u <= 1;
        }

        public double calculateAvailableSpace(DetectedSample currentSample, ArrayList<DetectedSample> samplesList) {
            double closest_distance = Double.MAX_VALUE;
            for (DetectedSample sample: samplesList){
                if (currentSample.rect.center.equals(sample.rect.center)){
                    continue;
                }
                if(intersectsSample(currentSample, sample)){
                    closest_distance = Math.min(distance(currentSample.rect.center, sample.rect.center), closest_distance);
                }
            }
            if (closest_distance == Double.MAX_VALUE){
                return 0;
            }
            return closest_distance;
        }
        public void sortSamplesBySpace(){
            ArrayList<DetectedSample> samplesList = new ArrayList<>(queue);

            for (int i = 0; i < samplesList.size(); i++) {
                DetectedSample currentSample = samplesList.get(i);
                double separation = calculateAvailableSpace(currentSample, samplesList);
                queue.remove(currentSample);
                currentSample.separation = separation;
                currentSample.score -= 50 * separation;
                queue.add(currentSample);
                if(currentSample.type.equals("Yellow")){
                    valid_options.remove(currentSample);
                    valid_options.add(currentSample);
                }
                // Track the point with the largest nearest distance
            }

        }

        public DetectedSample getSample(){
            if(valid_options.peek() != null){
                assert queue.peek() != null;
                if(valid_options.peek().area > approx_sample_size * 1.5){
                    return null;
                }
            }
            return valid_options.peek();
        }

    }
    public static Scalar lowerBlue = new Scalar(0, 100, 100);
    public static Scalar upperBlue = new Scalar(50, 255, 255);
    public static Scalar lowerRed1 =  new Scalar(115, 0, 0);
    public static Scalar upperRed1 =  new Scalar(160.1, 255, 250);
    public static Scalar lowerYellow = new Scalar(60, 100, 100);
    public static Scalar upperYellow = new Scalar(100, 255, 255);
    private double angle = 0;
    private double slide = 0;
    private double displacement = 0;
    private boolean yellow_sample = false;
    public boolean red = true;
    public boolean blue = true;
    double approx_sample_size = 70000;
    private boolean inside_pick = false;
    Mat blueMask = new Mat(), redMask = new Mat(), yellowMask = new Mat(), hsvFrame = new Mat(), outputFrame = new Mat();
    Map<String, Mat> masks;
    public TheBigBrainAlgorithm(){
        masks = new HashMap<>();
        masks.put("Yellow", yellowMask);
        masks.put("Red", redMask);
        masks.put("Blue", blueMask);
    }


    private static double getDistance(Point p1, Point p2) {
        return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
    }
    public static int getSlopeSignFromPoints(RotatedRect rotatedRect) {
        // Array to hold the 4 corner points of the rectangle
        Point[] rectPoints = new Point[4];
        rotatedRect.points(rectPoints);  // Fills the rectPoints array with the corner points

        // Find the two points that correspond to the width edge
        // The width edge will be the pair with the smallest distance between them.
        double minDistance = Double.MAX_VALUE;
        Point widthPoint1 = null;
        Point widthPoint2 = null;

        for (int i = 0; i < 4; i++) {
            for (int j = i + 1; j < 4; j++) {
                double distance = getDistance(rectPoints[i], rectPoints[j]);
                if (distance < minDistance) {
                    minDistance = distance;
                    widthPoint1 = rectPoints[i];
                    widthPoint2 = rectPoints[j];
                }
            }
        }

        // Calculate the slope using the two width points
        double slope = (widthPoint2.y - widthPoint1.y) / (widthPoint2.x - widthPoint1.x);

        // Determine if the slope is positive or negative
        if (slope > 0) {
            return -1;
        } else if (slope < 0) {
            return 1;
        } else {
            return 1;  // In case the slope is exactly zero (horizontal line)
        }
    }
    public void useInsidePick(boolean on){
        inside_pick = on;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Step 1: Convert the frame to HSV color space

        Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);
        Core.inRange(hsvFrame, lowerRed1, upperRed1, redMask);
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);
        // Step 3: Find contours for each mask
        String[] colors;
        if(red && blue){
            colors = new String[]{"Red", "Blue", "Yellow"};
        }
        else if(red){
            colors = new String[]{"Red", "Yellow"};
        }else if(blue){
            colors = new String[]{"Blue", "Yellow"};
        }else{
            colors = new String[]{"Yellow"};
        }

        outputFrame = input.clone();

        // Get the center of the frame
        double frameCenterX = 100;
        double frameCenterY = input.height() / 2.0;

        // Variables to track the closest contour for each color

        RotatedRect closestYellow;

        DetectedSampleList sampleList = new DetectedSampleList();

        // Loop through each color mask
        for (int i = 0; i < colors.length; i++) {
            List<MatOfPoint> contours = new ArrayList<>();
            Scalar color = new Scalar(0,0,0);
            if(colors[i].equals("Yellow")){
                color = new Scalar(100,100,0);
            }else if(colors[i].equals("Red")){
                color = new Scalar(100,0,0);
            }else if(colors[i].equals("Blue")){
                color = new Scalar(0,0,100);
            }
            Imgproc.findContours(masks.get(colors[i]), contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            // Step 4: Process each contour for the current color
            for (MatOfPoint contour : contours) {
                if (Imgproc.contourArea(contour) > approx_sample_size/3 && Imgproc.contourArea(contour) < 1.5 * approx_sample_size) {  // Filter small objects
                    // Step 5: Get the minimum area rectangle around the contour
                    MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                    RotatedRect rect = Imgproc.minAreaRect(contour2f);
                    // Step 6: Calculate the center of the rectangle
                    Point rectCenter = rect.center;
                    double area = rect.size.area();
                    double lateral_distance = rectCenter.x - frameCenterX;
                    double linear_distance = rectCenter.y - frameCenterY;
                    DetectedSample sample = new DetectedSample(colors[i], rect, area, lateral_distance, linear_distance);
                    sampleList.addSample(sample);
                    // Step 9: Check if this is the closest contour for the current color,
                    // prioritize smaller rectangles if the distances are comparable
                }
            }
        }
        sampleList.sortSamplesBySpace();
        DetectedSample yellowSample = sampleList.getSample();

        for(DetectedSample sample: sampleList.queue){
            if (sample.equals(yellowSample)) {
                continue;
            }
            Point[] points = new Point[4];
            sample.rect.points(points);
            for (int j = 0; j < 4; j++) {
                Imgproc.line(outputFrame, points[j], points[(j + 1) % 4], new Scalar(0,0,0), 2);
                Imgproc.putText(outputFrame, "Score: " + String.format("%.2f", sample.score) + " ticks",
                        new Point(sample.rect.center.x, sample.rect.center.y + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(200, 200, 200), 2);

            }
        }



        frameCenterX = outputFrame.width() / 2.0;

        if (yellowSample != null) {
            closestYellow = yellowSample.rect;
            double lateralDistance = closestYellow.center.x - frameCenterX;
            double linear_distance = closestYellow.center.y - frameCenterY;
            angle = closestYellow.angle;
            boolean rotate = inside_pick ? closestYellow.size.width > closestYellow.size.height :  closestYellow.size.width < closestYellow.size.height;
            if(rotate){
                angle = 90 - angle;
            }
            angle *= getSlopeSignFromPoints(closestYellow) * -1;
            slide = getSlidePosition(lateralDistance);
            displacement =  getRobotDisplacement(linear_distance);
            yellow_sample = true;
            Scalar color = new Scalar(0,0,0);

            if(yellowSample.type.equals("Yellow")){
                color = new Scalar(255,255,0);
            }else if(yellowSample.type.equals("Red")){
                color = new Scalar(255,0,0);
            }else if(yellowSample.type.equals("Blue")){
                color = new Scalar(0,0,255);
            }
            drawRect(outputFrame, closestYellow, color, "Sample", angle, sampleList.getSample().score,sampleList.getSample().area, sampleList.getSample().separation);
        }else{
            yellow_sample = false;
        }

        Imgproc.putText(outputFrame, "Sample: " + yellow_sample + " px",
                new Point(0,20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(200, 200, 200), 2);

        // Return the frame with the bounding boxes and angles drawn
        return outputFrame;
    }
    public double[] redPosition(){
        return new double[]{0,0,0};
    } public double[] bluePosition(){
        return new double[]{0,0,0};
    } public double[] yellowPosition(){
        return new double[]{angle, slide, displacement};
    }
    public boolean yellowSampleDetected(){
        return yellow_sample;
    }
    public boolean redSampleDetected(){
        return false;
    }
    public boolean blueSampleDetected(){
        return false;
    }

    private void drawRect(Mat frame, RotatedRect rect, Scalar color, String label, double angle, double score, double area, double separation) {
        // Draw the rectangle
        Point[] points = new Point[4];
        rect.points(points);
        for (int j = 0; j < 4; j++) {
            Imgproc.line(frame, points[j], points[(j + 1) % 4], color, 2);
        }

        // Calculate lateral and linear distances from the frame's center
        double frameCenterX = frame.width() / 2.0;
        double frameCenterY = frame.height() / 2.0;

        // Lateral distance (horizontal displacement on the x-axis)
        double lateralDistance = rect.center.x - frameCenterX;

        // Linear distance (Euclidean distance from the frame center)
        double linearDistance = rect.center.y - frameCenterY;

        Imgproc.putText(frame, label + " - Angle: " + -1 * (int) angle, rect.center, Imgproc.FONT_HERSHEY_SIMPLEX , 0.6, new Scalar(200,200,200), 2);
        if(debug){
            Imgproc.putText(frame, "Slide Position: " + String.format("%.2f", lateralDistance) + " px",
                    new Point(rect.center.x, rect.center.y + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(200, 200, 200), 2);
            Imgproc.putText(frame, "Linear: " + String.format("%.2f", linearDistance) +  " px",
                    new Point(rect.center.x, rect.center.y + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(200, 200, 200), 2);

        }else{
            Imgproc.putText(frame, "Slide Position: " + String.format("%.2f", getSlidePosition(lateralDistance)) + " ticks",
                    new Point(rect.center.x, rect.center.y + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(200, 200, 200), 2);
            Imgproc.putText(frame, "Linear: " + String.format("%.2f", getRobotDisplacement(linearDistance)) + " in",
                    new Point(rect.center.x, rect.center.y + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(200, 200, 200), 2);

        }
       Imgproc.putText(frame, "Width: " + String.format("%.2f", rect.size.width) + " px",
                new Point(rect.center.x, rect.center.y + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(200, 200, 200), 2);
        Imgproc.putText(frame, "Height: " + String.format("%.2f", rect.size.height) + " px",
                new Point(rect.center.x, rect.center.y + 80), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(200, 200, 200), 2);
        Imgproc.putText(frame, "Score: " + String.format("%.2f", score) + " pts",
                new Point(rect.center.x, rect.center.y + 100), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(200, 200, 200), 2);
        Imgproc.putText(frame, "Area: " + String.format("%.2f", area) + " pts",
                new Point(rect.center.x, rect.center.y + 120), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(200, 200, 200), 2);
        Imgproc.putText(frame, "Separation: " + String.format("%.2f", separation) + " pts",
                new Point(rect.center.x, rect.center.y + 140), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(200, 200, 200), 2);

    }
    public double getSlidePosition(double pixels){
        double m = (double) (590 - 500) /(-74 + 248);
        return  (int) (m * (pixels + 74) + 590);
    }
    public double getRobotDisplacement(double pixels){
        return pixels/130.0 * 1.5;
    }

}


