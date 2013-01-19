package team79.smartdashboard.extension.krunchcv;

import com.googlecode.javacv.CanvasFrame;
import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_core.CvSize;
import com.googlecode.javacv.cpp.opencv_core.IplImage;
import com.googlecode.javacv.cpp.opencv_imgproc;
import com.googlecode.javacv.cpp.opencv_imgproc.IplConvKernel;
import edu.wpi.first.smartdashboard.camera.WPICameraExtension;
import edu.wpi.first.smartdashboard.robot.Robot;
import edu.wpi.first.wpijavacv.DaisyExtensions;
import edu.wpi.first.wpijavacv.WPIBinaryImage;
import edu.wpi.first.wpijavacv.WPIColor;
import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIContour;
import edu.wpi.first.wpijavacv.WPIImage;
import edu.wpi.first.wpijavacv.WPIPoint;
import edu.wpi.first.wpijavacv.WPIPolygon;
import edu.wpi.first.wpilibj.networking.NetworkListener;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.Scanner;
import java.util.TreeMap;
import javax.imageio.ImageIO;
import javax.swing.JOptionPane;

/**
 *
 * @author sebastian
 */
public class KrunchCVWidget extends WPICameraExtension implements NetworkListener
{
    public static final String NAME = "Krunch Target Tracker"; // Name of widget in View->Add list
    private WPIColor alignedColor = new WPIColor(0, 255, 0); // Color of overlay when camera is aligned with goal
    private WPIColor unalignedColor = new WPIColor(255, 0, 0); // Color of overlay when camera is unaligned with goal
    private WPIColor centerPointColor = new WPIColor(255, 255, 0); // Color of polygon center point
    
    // Constants that need to be tuned
    private static final double kNearlyHorizontalSlope = Math.tan(Math.toRadians(20)); // Slope of an acceptable horizontal line in degrees
    private static final double kNearlyVerticalSlope = Math.tan(Math.toRadians(90-20)); // Slope of an acceptable vertical line in degrees
    private static final int kMinWidth = 20; // Contour ratio min width
    private static final int kMaxWidth = 200; // Contour ration max width
    private static final double kRangeOffset = 0.0; // Offset for adjusting range
    private static final int kHoleClosingIterations = 9; // Number of iterations of morphology operation
    
    private static final double kShooterOffsetDeg = 0.0; // Offset for shooter
    private static final double kHorizontalFOVDeg = 40.0; // Horizontal field of view of camera

    private static final double kVerticalFOVDeg = 480.0/640.0*kHorizontalFOVDeg; // Vertical field of view of camera *(FROM 640x480 images)
    private static final double kCameraHeightIn = 4.0 + 3.0/8.0; // Height of camera from ground in inches
    private static final double kCameraPitchDeg = 20.0; // Camera pitch degree (as in pitch, roll, yaw)
    private static final double kTopTargetHeightIn = 53.5;

    // Constants that pertain to HSV threshold value file
    private static final String DEFAULT_CSV_FILENAME = "KrunchCVSettings.txt";
    private static final String s_lineSeparator = System.getProperty("line.separator");
    
    // SmartDashboard Key Values
    private static final String brightKey = "BRIGHTNESS";
    private static final String contrastKey = "CONTRAST";
    private static final String hueMinKey = "HUE MIN";
    private static final String hueMaxKey = "HUE MAX";
    private static final String satMinKey = "SAT MIN";
    private static final String satMaxKey = "SAT MAX";
    private static final String valMinKey = "VAL MIN";
    private static final String valMaxKey = "VAL MAX";
    private static final String goalAlignToleranceKey = "G.O.A.T.";
    
    private static final String saveKey = "save";
    
    private double goalOverlayAlignTolerance;
    
    private double brightness, contrast;
    private double hueMin, hueMax;
    private double satMin, satMax;
    private double valMin, valMax;
    
    private boolean saving = false;
    
    private TreeMap<Double, Double> rangeTable;

    private boolean m_debugMode = false;

    // Store JavaCV temporaries as members to reduce memory management during processing
    private CvSize size = null;
    private WPIContour[] contours;
    private ArrayList<WPIPolygon> polygons;
    private IplConvKernel morphKernel;
    private IplImage bin; // Container for binary image
    private IplImage hsv;
    private IplImage hue1, hue2;
    private IplImage sat1, sat2;
    private IplImage val1, val2;
    private WPIPoint linePt1;
    private WPIPoint linePt2;
    private int horizontalOffsetPixels;

    public KrunchCVWidget()
    {
        this(false);
    }

    public KrunchCVWidget(boolean debug)
    {
        m_debugMode = debug;
        morphKernel = IplConvKernel.create(3, 3, 1, 1, opencv_imgproc.CV_SHAPE_RECT, null);

        rangeTable = new TreeMap<Double,Double>();
        //rangeTable.put(110.0, 3800.0+kRangeOffset);
        //rangeTable.put(120.0, 3900.0+kRangeOffset);
        //rangeTable.put(130.0, 4000.0+kRangeOffset);
        rangeTable.put(140.0, 3434.0+kRangeOffset);
        rangeTable.put(150.0, 3499.0+kRangeOffset);
        rangeTable.put(160.0, 3544.0+kRangeOffset);
        rangeTable.put(170.0, 3574.0+kRangeOffset);
        rangeTable.put(180.0, 3609.0+kRangeOffset);
        rangeTable.put(190.0, 3664.0+kRangeOffset);
        rangeTable.put(200.0, 3854.0+kRangeOffset);
        rangeTable.put(210.0, 4034.0+kRangeOffset);
        rangeTable.put(220.0, 4284.0+kRangeOffset);
        rangeTable.put(230.0, 4434.0+kRangeOffset);
        rangeTable.put(240.0, 4584.0+kRangeOffset);
        rangeTable.put(250.0, 4794.0+kRangeOffset);
        rangeTable.put(260.0, 5034.0+kRangeOffset);
        rangeTable.put(270.0, 5234.0+kRangeOffset);
        
        try 
        {
            // Load HSV Threshold values from CSV File
            this.loadSettingsFile();
        } 
        catch (IOException ex) 
        {
            handleError(ex);
        }
        
        // Init NetworkListener for changed keys
        Robot.getTable().addListenerToAll(this);
        
        DaisyExtensions.init();
    }
    
    private void loadSettingsFile() throws IOException
    {
        try {
            FileReader fr = new FileReader(DEFAULT_CSV_FILENAME);
            BufferedReader br = new BufferedReader(fr);
            String buffer;
            while((buffer = br.readLine()) != null)
            {
                // Remove unwanted line separators
                buffer = buffer.replace(s_lineSeparator, "");
                
                String key = "";
                double numValue = 0.0;
                
                // Get key and value
                if(buffer.contains(","))
                {
                    String[] temp = buffer.split(",");
                    key = temp[0];
                    numValue = Double.valueOf(temp[1]);
                }
                else if(buffer.contains(", "))
                {
                    String[] temp = buffer.split(", ");
                    key = temp[0];
                    numValue = Double.valueOf(temp[1]);
                }
                
                // Change corresponding value
                if(brightKey.equals(key))
                {
                    brightness = numValue;
                    Robot.getTable().putDouble(key, numValue);
                }
                else if(contrastKey.equals(key))
                {
                    contrast = numValue;
                    Robot.getTable().putDouble(key, numValue);
                }
                else if(hueMinKey.equals(key))
                {
                    hueMin = numValue;
                    Robot.getTable().putDouble(key, numValue);
                }
                else if(hueMaxKey.equals(key))
                {
                    hueMax = numValue;
                    Robot.getTable().putDouble(key, numValue);
                }
                else if(satMinKey.equals(key))
                {
                    satMin = numValue;
                    Robot.getTable().putDouble(key, numValue);
                }
                else if(satMaxKey.equals(key))
                {
                    satMax = numValue;
                    Robot.getTable().putDouble(key, numValue);
                }
                else if(valMinKey.equals(key))
                {
                    valMin = numValue;
                    Robot.getTable().putDouble(key, numValue);
                }
                else if(valMaxKey.equals(key))
                {
                    valMax = numValue;
                    Robot.getTable().putDouble(key, numValue);
                }
                else if(goalAlignToleranceKey.equals(key))
                {
                    goalOverlayAlignTolerance = numValue;
                    Robot.getTable().putDouble(key, numValue);
                }
            }
            fr.close();
            
        } catch (FileNotFoundException ex) {
            try {
                // Create new file and add default values
                FileWriter fw = new FileWriter(DEFAULT_CSV_FILENAME);
                fw.write(brightKey + ", 0" + s_lineSeparator +
                            contrastKey + ", 0" + s_lineSeparator +
                            hueMinKey + ", 0" + s_lineSeparator +
                            hueMaxKey + ", 0" + s_lineSeparator +
                            satMinKey + ", 0" + s_lineSeparator +
                            satMaxKey + ", 0" + s_lineSeparator +
                            valMinKey + ", 0" + s_lineSeparator +
                            valMaxKey + ", 0" + s_lineSeparator +
                            goalAlignToleranceKey + ", 0" + s_lineSeparator);
                
                fw.flush();
                fw.close();
                
                // Set NetworkTable values to 0
                Robot.getTable().putDouble(brightKey, 0);
                Robot.getTable().putDouble(contrastKey, 0);
                Robot.getTable().putDouble(hueMinKey, 0);
                Robot.getTable().putDouble(hueMaxKey, 0);
                Robot.getTable().putDouble(satMinKey, 0);
                Robot.getTable().putDouble(satMaxKey, 0);
                Robot.getTable().putDouble(valMinKey, 0);
                Robot.getTable().putDouble(valMaxKey, 0);
                Robot.getTable().putDouble(goalAlignToleranceKey, 0);
                
            } catch (IOException iEx) {
                handleError(iEx);
            }
        }
    }
    
    private void saveSettingsFile()
    {
        Thread saveThread;
        saveThread = new Thread(){
            @Override
            public void run(){
                try 
                {
                    // Create new file and add default values
                    FileWriter fw = new FileWriter(DEFAULT_CSV_FILENAME);
                    
                    // Write keys in order
                    fw.write(brightKey + ", " + Double.toString(brightness) + s_lineSeparator +
                                contrastKey + ", " + Double.toString(contrast) + s_lineSeparator +
                                hueMinKey + ", " + Double.toString(hueMin) + s_lineSeparator +
                                hueMaxKey + ", " + Double.toString(hueMax) + s_lineSeparator +
                                satMinKey + ", " + Double.toString(satMin) + s_lineSeparator +
                                satMaxKey + ", " + Double.toString(satMax) + s_lineSeparator +
                                valMinKey + ", " + Double.toString(valMin) + s_lineSeparator +
                                valMaxKey + ", " + Double.toString(valMax) + s_lineSeparator +
                                goalAlignToleranceKey + ", " + Double.toString(goalOverlayAlignTolerance) + s_lineSeparator);

                    fw.flush();
                    fw.close();
                } 
                catch (FileNotFoundException ex) 
                {
                    handleError(ex);
                }
                catch (IOException iEx)
                {
                    handleError(iEx);
                }
                
                // State the save is finished
                Robot.getPreferences().putBoolean(saveKey, false);
                saving = false;
              }
        };
        
        // Start Thread
        saveThread.start();
    }
    
    private void updateLocalSettings()
    {
        // Assign Settings Values from SmartDashboard.
        brightness = Robot.getTable().getDouble(brightKey);
        contrast = Robot.getTable().getDouble(contrastKey);
        hueMin = Robot.getTable().getDouble(hueMinKey);
        hueMax = Robot.getTable().getDouble(hueMaxKey);
        satMin = Robot.getTable().getDouble(satMinKey);
        satMax = Robot.getTable().getDouble(satMaxKey);
        valMin = Robot.getTable().getDouble(valMinKey);
        valMax = Robot.getTable().getDouble(valMaxKey);
        
        goalOverlayAlignTolerance = Robot.getTable().getDouble(goalAlignToleranceKey);
    }
    
    public double getRPMsForRange(double range)
    {
        double lowKey = -1.0;
        double lowVal = -1.0;
        for( double key : rangeTable.keySet() )
        {
            if( range < key )
            {
                double highVal = rangeTable.get(key);
                if( lowKey > 0.0 )
                {
                    double m = (range-lowKey)/(key-lowKey);
                    return lowVal+m*(highVal-lowVal);
                }
                else
                {
                    return highVal;
                }
                    
            }
            lowKey = key;
            lowVal = rangeTable.get(key);
        }

        return 5234.0+kRangeOffset;
    }
    
    @Override
    public WPIImage processImage(WPIColorImage rawImage)
    {
        double heading = 0.0;
        
        // Get the current heading of the robot first
        if( !m_debugMode )
        {
            try
            {
                heading = Robot.getTable().getDouble("Heading");
            }
            catch( NoSuchElementException e)
            {
            }
            catch( IllegalArgumentException e )
            {
            }
        }

        // If size hasn't been initialized yet
        if( size == null || size.width() != rawImage.getWidth() || size.height() != rawImage.getHeight() )
        {
            size = opencv_core.cvSize(rawImage.getWidth(),rawImage.getHeight());
            bin = IplImage.create(size, 8, 1); // Binary image container
            hsv = IplImage.create(size, 8, 3); // CvSize, depth, number of channels
            hue1 = IplImage.create(size, 8, 1);
            hue2 = IplImage.create(size, 8, 1);
            sat1 = IplImage.create(size, 8, 1);
            sat2 = IplImage.create(size, 8, 1);
            val1 = IplImage.create(size, 8, 1);
            val2 = IplImage.create(size, 8, 1);
            horizontalOffsetPixels =  (int)Math.round(kShooterOffsetDeg*(size.width()/kHorizontalFOVDeg));
            
            // Line points for line that goes down the middle of the image when outputed on the dashboard
            linePt1 = new WPIPoint(size.width()/2+horizontalOffsetPixels,size.height()-1);
            linePt2 = new WPIPoint(size.width()/2+horizontalOffsetPixels,0);
        }
        // Get the raw IplImages for OpenCV
        IplImage input = DaisyExtensions.getIplImage(rawImage);

        // Convert to HSV color space
        opencv_imgproc.cvCvtColor(input, hsv, opencv_imgproc.CV_BGR2HSV);
        opencv_core.cvSplit(hsv, hue1, sat1, val1, null);       // Init to first IplImage

        // Threshold each component separately
        // Hue
        // NOTE: Red is at the end of the color space, so you need to OR together
        // a thresh and inverted thresh in order to get points that are red
        
        // INV is for the Upper thresholds. Note that THE ORDER YOU CALL THE FUNCTIONS MATTER
        // because if you call INV second, you will overwrite the hue1 init image.
        
        // Take input of hue1(init image), output of function to hue2(Output is a binary image)
        opencv_imgproc.cvThreshold(hue1, hue2, hueMax, 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        opencv_imgproc.cvThreshold(hue1, hue1, hueMin, 255, opencv_imgproc.CV_THRESH_BINARY);

        // Saturation
        opencv_imgproc.cvThreshold(sat1, sat2, satMax, 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        opencv_imgproc.cvThreshold(sat1, sat1, satMin, 255, opencv_imgproc.CV_THRESH_BINARY);

        // Value
        opencv_imgproc.cvThreshold(val1, val2, valMax, 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        opencv_imgproc.cvThreshold(val1, val1, valMin, 255, opencv_imgproc.CV_THRESH_BINARY);

        
        // Combine the results to obtain our binary image which should for the most
        // part only contain pixels that we care about (Bin is initialized with cvAnd of hue1 and hue2
        // and then is bitwise anded incrementally with each new thresholded image
          opencv_core.cvAnd(hue1, hue2, bin, null);
          opencv_core.cvAnd(sat1, bin, bin, null);
          opencv_core.cvAnd(sat2, bin, bin, null);
          opencv_core.cvAnd(val1, bin, bin, null);
          opencv_core.cvAnd(val2, bin, bin, null);

        // Uncomment the next two lines to see the raw binary image
        //CanvasFrame result = new CanvasFrame("binary");
        //result.showImage(bin.getBufferedImage());
        
        // Fill in any gaps using binary morphology
        // Changing the 5th parameter changes the method, and changing the 6th parameter changes the number of iterations
        // of the pixel extrapolation process.
        opencv_imgproc.cvMorphologyEx(bin, bin, null, morphKernel, opencv_imgproc.CV_MOP_CLOSE, kHoleClosingIterations);

        // Uncomment the next two lines to see the image post-morphology
        //CanvasFrame result2 = new CanvasFrame("morph");
        //result2.showImage(bin.getBufferedImage());

        // Find contours
        WPIBinaryImage binWpi = DaisyExtensions.makeWPIBinaryImage(bin);
        contours = DaisyExtensions.findConvexContours(binWpi);

        polygons = new ArrayList<WPIPolygon>();
        for (WPIContour c : contours)
        {
            double ratio = ((double) c.getHeight()) / ((double) c.getWidth());
            if (ratio < 1.0 && ratio > 0.5 && c.getWidth() > kMinWidth && c.getWidth() < kMaxWidth)
            {
                polygons.add(c.approxPolygon(20));
            }
        }

        WPIPolygon square = null;
        int highest = Integer.MAX_VALUE;

        for (WPIPolygon p : polygons)
        {
            if (p.isConvex() && p.getNumVertices() == 4)
            {
                // We passed the first test...we fit a rectangle to the polygon
                // Now do some more tests

                WPIPoint[] points = p.getPoints();
                // We expect to see a top line that is nearly horizontal, and two side lines that are nearly vertical
                int numNearlyHorizontal = 0;
                int numNearlyVertical = 0;
                for( int i = 0; i < 4; i++ )
                {
                    double dy = points[i].getY() - points[(i+1) % 4].getY(); // Change in Y from one point to the next
                    double dx = points[i].getX() - points[(i+1) % 4].getX(); // Change in X from one point to the next
                    double slope = Double.MAX_VALUE;
                    // If slope not 0, in other words not perfectly horizontal
                    if( dx != 0 ) 
                    {
                        // Find slope of line
                        slope = Math.abs(dy/dx);
                    } 

                    // Increment number of horizontal or vertical sides depending on if the slope is
                    // closer to being horizontal or if the slope is closer to being verticle.
                    if( slope < kNearlyHorizontalSlope )
                    {
                        ++numNearlyHorizontal;
                    }
                    else if( slope > kNearlyVerticalSlope )
                    {
                        ++numNearlyVertical;
                    }
                }

                // Since we assume the top line is horizontal, the funciton only requires that
                // we have 1 nearly horizontal side and 2 nearly verticle sides to consider it
                // a target.
                if(numNearlyHorizontal >= 1 && numNearlyVertical == 2)
                {
                    // Draw a polygon overlay on the image
                    rawImage.drawPolygon(p, WPIColor.BLUE, 2);

                    // Get center of polygon
                    int pCenterX = (p.getX() + (p.getWidth() / 2));
                    int pCenterY = (p.getY() + (p.getHeight() / 2));

                    // Draw a large point emphasizing the center of the polygon
                    rawImage.drawPoint(new WPIPoint(pCenterX, pCenterY), centerPointColor, 5);
                    
                    // Picks the highest target
                    // The origin of the coordinate system is at the top-left of the image,
                    // which is why the comparison is less than. The height values get smaller
                    // when they are higher up in reality.
                    if (pCenterY < highest) // Because coord system is funny
                    {
                        square = p;
                        highest = pCenterY;
                    }
                }
            }
            else
            {
                // Not an acceptable target
                rawImage.drawPolygon(p, WPIColor.YELLOW, 1);
            }
        }

        // If a target has been found
        if (square != null)
        {
            double x = square.getX() + (square.getWidth() / 2);
            x = (2 * (x / size.width())) - 1;
            double y = square.getY() + (square.getHeight() / 2);
            y = -((2 * (y / size.height())) - 1);

            // Find azimuth (horizontal degrees needed to line up with target). This is given as -180 being completely left,
            // +180 being completely right, and 0 being completely lined up.
            double azimuth = this.boundAngle0to180DegreesWithDirection(x*kHorizontalFOVDeg/2.0 + heading - kShooterOffsetDeg);
            double range = (kTopTargetHeightIn-kCameraHeightIn)/Math.tan((y*kVerticalFOVDeg/2.0 + kCameraPitchDeg)*Math.PI/180.0);
            double rpms = getRPMsForRange(range);

            if (!m_debugMode)
            {
                // Outputs values to the dashboard
                Robot.getTable().beginTransaction();
                Robot.getTable().putBoolean("found", true);
                Robot.getTable().putDouble("azimuth", azimuth);
                Robot.getTable().putDouble("range", range);
                Robot.getTable().putDouble("rpms", rpms);
                Robot.getTable().endTransaction();
            } else
            {
                System.out.println("Target found");
                System.out.println("x: " + x);
                System.out.println("y: " + y);
                System.out.println("azimuth: " + azimuth);
                System.out.println("range: " + range);
                System.out.println("rpms: " + rpms);
            }
            
            // Draw outline around highest goal
            double centerX = square.getX() + square.getWidth()/2;
            if(centerX >= linePt1.getX()-goalOverlayAlignTolerance && centerX <= linePt1.getX()+goalOverlayAlignTolerance)
            {
                // Draw Aligned Outline
                rawImage.drawPolygon(square, alignedColor, 7);
            }
            else
            {
                // Draw Unaligned Outline
                rawImage.drawPolygon(square, unalignedColor, 7);
            }
            
        } else
        {

            if (!m_debugMode)
            {
                Robot.getTable().putBoolean("found", false);
            } else
            {
                System.out.println("Target not found");
            }
        }

        // Draw a crosshair (line down the middle)
        rawImage.drawLine(linePt1, linePt2, alignedColor, 2);

        DaisyExtensions.releaseMemory();

        //System.gc();

        // Look to see if button was pressed to save settings
        try{
            if(Robot.getPreferences().getBoolean(saveKey) && !saving){
                this.saveSettingsFile();
                saving = true;
            }
        }
        catch( NoSuchElementException e)
        {
        }
        catch( IllegalArgumentException e )
        {
        }
        
        return rawImage;
    }

    private double boundAngle0to180DegreesWithDirection(double angle)
    {
        // Find coterminal angle
        while(angle >= 360.0)
        {
            angle -= 360.0;
        }
        while(angle < 0.0)
        {
            angle += 360.0;
        }
        
        // Bound left and right sides of circle to 180 degrees.
        // The -1 indicates that it is left
        if(angle > 180.0)
        {
            angle = (360.0 - angle) * -1;
        }
        
        return angle;
        
    }

    private void handleError(Exception e)
    {
        e.printStackTrace();
        JOptionPane.showMessageDialog(null,
            "An error occurred when attempting to "
            + "open the CSV file for writing. ",
            "Unable to Open CSV File",
            JOptionPane.ERROR_MESSAGE);
    }
    
    @Override
    public void valueChanged(String key, Object value) {
        
        // If key value was changed concerning this widget
        if(brightKey.equals(key) || contrastKey.equals(key) || hueMinKey.equals(key) ||
                hueMaxKey.equals(key) || satMinKey.equals(key) || satMaxKey.equals(key) || 
                valMinKey.equals(key) || valMaxKey.equals(key) || goalAlignToleranceKey.equals(key))
        {
            // Reload the values into the widget
            this.updateLocalSettings();
        }
    }

    @Override
    public void valueConfirmed(String key, Object value) {
        valueChanged(key, value);
    }

    @Override
    public void disconnect() {
        // Save HSV Threshold values when widget is removed.
        this.saveSettingsFile();
    }
    
    
    
    
    public static void main(String[] args)
    {
        if (args.length == 0)
        {
            System.out.println("Usage: Arguments are paths to image files to test the program on");
            return;
        }

        // Create the widget
        KrunchCVWidget widget = new KrunchCVWidget(true);

        long totalTime = 0;
        for (int i = 0; i < args.length; i++)
        {
            // Load the image
            WPIColorImage rawImage = null;
            try
            {
                rawImage = new WPIColorImage(ImageIO.read(new File(args[i%args.length])));
            } catch (IOException e)
            {
                System.err.println("Could not find file!");
                return;
            }
            
            //shows the raw image before processing to eliminate the possibility
            //that both may be the modified image.
            CanvasFrame original = new CanvasFrame("Raw");
            original.showImage(rawImage.getBufferedImage());

            WPIImage resultImage = null;

            // Process image
            long startTime, endTime;
            startTime = System.nanoTime();
            resultImage = widget.processImage(rawImage);
            endTime = System.nanoTime();

            // Display results
            totalTime += (endTime - startTime);
            double milliseconds = (double) (endTime - startTime) / 1000000.0;
            System.out.format("Processing took %.2f milliseconds%n", milliseconds);
            System.out.format("(%.2f frames per second)%n", 1000.0 / milliseconds);
            
            CanvasFrame result = new CanvasFrame("Result");
            result.showImage(resultImage.getBufferedImage());

            System.out.println("Waiting for ENTER to continue to next image or exit...");
            Scanner console = new Scanner(System.in);
            console.nextLine();

            if (original.isVisible())
            {
                original.setVisible(false);
                original.dispose();
            }
            if (result.isVisible())
            {
                result.setVisible(false);
                result.dispose();
            }
        }

        double milliseconds = (double) (totalTime) / 1000000.0 / (args.length);
        System.out.format("AVERAGE:%.2f milliseconds%n", milliseconds);
        System.out.format("(%.2f frames per second)%n", 1000.0 / milliseconds);
        System.exit(0);
    }
}
