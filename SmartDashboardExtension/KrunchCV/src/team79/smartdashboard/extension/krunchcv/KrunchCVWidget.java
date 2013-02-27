package team79.smartdashboard.extension.krunchcv;

import com.googlecode.javacv.CanvasFrame;
import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_core.CvSize;
import com.googlecode.javacv.cpp.opencv_core.IplImage;
import com.googlecode.javacv.cpp.opencv_imgproc;
import com.googlecode.javacv.cpp.opencv_imgproc.IplConvKernel;
import edu.wpi.first.smartdashboard.camera.WPICameraExtension;
import edu.wpi.first.smartdashboard.properties.IntegerProperty;
import edu.wpi.first.smartdashboard.properties.Property;
import edu.wpi.first.smartdashboard.properties.StringProperty;
import edu.wpi.first.smartdashboard.robot.Robot;
import edu.wpi.first.wpijavacv.DaisyExtensions;
import edu.wpi.first.wpijavacv.WPIBinaryImage;
import edu.wpi.first.wpijavacv.WPIColor;
import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIContour;
import edu.wpi.first.wpijavacv.WPIImage;
import edu.wpi.first.wpijavacv.WPIPoint;
import edu.wpi.first.wpijavacv.WPIPolygon;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Scanner;
import javax.imageio.ImageIO;
import javax.swing.JOptionPane;

/**
 *
 * @author sebastian
 */
public class KrunchCVWidget extends WPICameraExtension implements ITableListener
{
    public static final String NAME = "Krunch Target Tracker"; // Name of widget in View->Add list
    private WPIColor alignedColor = new WPIColor(0, 255, 0); // Color of overlay when camera is aligned with goal
    private WPIColor unalignedColor = new WPIColor(255, 0, 0); // Color of overlay when camera is unaligned with goal
    private WPIColor centerPointColor = new WPIColor(255, 255, 0); // Color of polygon center point
    
    // Constants that need to be tuned
    private static final double kNearlyHorizontalSlope = Math.tan(Math.toRadians(20)); // Slope of an acceptable horizontal line in degrees
    private static final double kNearlyVerticalSlope = Math.tan(Math.toRadians(90-20)); // Slope of an acceptable vertical line in degrees
    private static final int kMinWidthRectGoals = 40; // Contour width ratios of rectangular goals
    private static final int kMaxWidthRectGoals = 200;
    private static final int kMinWidthPyramidHGoals = 20; // Contour width ratios of pyramid horizontal goals *NEEDS TO BE CHANGED
    private static final int kMaxWidthPyramidHGoals = 200;
    private static final int kMinWidthPyramidVGoals = 20; // Contour width ratios of pyramid vertical goals *NEEDS TO BE CHANGED
    private static final int kMaxWidthPyramidVGoals = 200;
    private static final int kHoleClosingIterations = 9; // Number of iterations of morphology operation
    
    private static final double kShooterOffsetDeg = 0.0; // Offset for shooter
    private static final double kHorizontalFOVDeg = 47.0; // Horizontal field of view of camera

    private static final double kVerticalFOVDeg = 480.0/640.0*kHorizontalFOVDeg; // Vertical field of view of camera *(FROM 640x480 images)

    
    // Widget Property keys
    private final String teamNumberKey = "Team Number";
    private final String ipKey = "Network Table IP";
    private final String portKey = "Network Table Port";
    private final String tableNameKey = "Table name";
    
    // Widget Properties
    public final IntegerProperty TEAM_NUMBER_PROPERTY = new IntegerProperty(this, teamNumberKey, 79);
    public final StringProperty IP_PROPERTY = new StringProperty(this, ipKey, "10.0.79.2");
    public final IntegerProperty PORT_PROPERTY = new IntegerProperty(this, portKey, 1735);
    public final StringProperty TABLE_NAME_PROPERTY = new StringProperty(this, tableNameKey); // Not really needed (as of now)
    
    
    // Constants that pertain to HSV threshold value file
    private static final String DEFAULT_CSV_FILENAME = "KrunchCVSettings.txt";
    private static final String s_lineSeparator = System.getProperty("line.separator");
    
    // SmartDashboard Key Values (DOUBLES ONLY)
    Map<String, Object> keyMap;
    
    private final String brightKey = "BRIGHTNESS";
    private final String contrastKey = "CONTRAST";
    private final String hueMinKey = "HUE MIN";
    private final String hueMaxKey = "HUE MAX";
    private final String satMinKey = "SAT MIN";
    private final String satMaxKey = "SAT MAX";
    private final String valMinKey = "VAL MIN";
    private final String valMaxKey = "VAL MAX";
    private final String goalAlignToleranceKey = "G.O.A.T.";
    private final String cameraHeightInchesKey = "Camera Height in Inches"; // Height of camera from ground in inches
    private final String shooterTiltedKey = "shooter tilted"; // Whether or not the shooter is tilted
    private final String cameraPitchDegLowKey = "Camera Pitch Degree Low"; // Low angle camera pitch degree
    private final String cameraPitchDegHighKey = "Camera Pitch Degree High"; // High angle camera pitch degree
    private final String topTargetHeightInchesKey = "Top Target Height Inches"; // Height of the top target
    private final String minWidthRectGoalsKey = "Min Width Rect Goals"; // Min width in pixels that vision will consider a goal
    private final String maxWidthRectGoalsKey = "Max Width Rect Goals"; // Max width in pixels that vision will consider a goal
    
    private static final String saveKey = "save"; // Boolean value
    
    private boolean shooterTilted = false;
    
    private boolean saving = false;

    private boolean m_debugMode = false;
    
    private CanvasFrame cf;

    private NetworkTable netTable = null;
    
    // Store JavaCV temporaries as members to reduce memory management during processing
    private CvSize size = null;
    private WPIContour[] contours;
    private ArrayList<WPIPolygon> rectGoalPolygons, pyramidGoalHPolygons, pyramidGoalVPolygons;
    private ArrayList<HorizontalPyramidRect> horizontalPyramidRects;
    private ArrayList<VerticalPyramidRect> verticalPyramidRects;
    private IplConvKernel morphKernel;
    private IplImage bin; // Container for binary image
    private IplImage hsv;
    private IplImage hue1, hue2;
    private IplImage sat1, sat2;
    private IplImage val1, val2;
    private WPIPoint linePt1, linePt2, linePt3, linePt4;
    private int horizontalOffsetPixels;

    public KrunchCVWidget()
    {
        this(false);
    }

    public KrunchCVWidget(boolean debug)
    {
        m_debugMode = debug;
        morphKernel = IplConvKernel.create(3, 3, 1, 1, opencv_imgproc.CV_SHAPE_RECT, null);
        
        cf = new CanvasFrame("Binary");
        
        // Create hashmap to store all values with keys
        // The second parameter is the default value
        keyMap = new HashMap<String, Object>();
        keyMap.put(brightKey, 0.0);
        keyMap.put(contrastKey, 0.0);
        keyMap.put(hueMinKey, 0.0);
        keyMap.put(hueMaxKey, 0.0);
        keyMap.put(satMinKey, 0.0);
        keyMap.put(satMaxKey, 0.0);
        keyMap.put(valMinKey, 0.0);
        keyMap.put(valMaxKey, 0.0);
        keyMap.put(goalAlignToleranceKey, 0.0);
        keyMap.put(cameraHeightInchesKey, 0.0);
        keyMap.put(cameraPitchDegLowKey, 0.0);
        keyMap.put(cameraPitchDegHighKey, 0.0);
        keyMap.put(topTargetHeightInchesKey, 0.0);
        keyMap.put(minWidthRectGoalsKey, 0.0);
        keyMap.put(maxWidthRectGoalsKey, 0.0);
        
        // Update Properties (Setup networktable info)
        this.updateFromProperties();

        
        try 
        {
            // Load Settings from CSV File
            this.loadSettingsFile();
        } 
        catch (Exception ex) 
        {
            handleCSVFileError(ex);
        }
        
        DaisyExtensions.init();
    }
    
    private void loadSettingsFile() throws Exception
    {
        try {
            FileReader fr = new FileReader(DEFAULT_CSV_FILENAME);
            BufferedReader br = new BufferedReader(fr);
            String buffer = "";
            while((buffer = br.readLine()) != null)
            {
                if(buffer != "")
                {
                    // Remove unwanted line separators
                    buffer = buffer.replace(s_lineSeparator, "");
                    String key = "";
                    Object objValue = null;

                    // Get key and value
                    String[] temp = null;
                    if(buffer.contains(",") && !buffer.contains(", "))
                    {
                        temp = buffer.split(",");
                        key = temp[0];
                    }
                    else if(buffer.contains(", "))
                    {
                        temp = buffer.split(", ");
                        key = temp[0];
                    }
                    
                    // Determine the correct type
                    if(temp[1].equals("true") || temp[1].equals("false"))
                    {
                        // Boolean
                        objValue = (temp[1].equals("true")) ? true : false;
                    }
                    else if(temp[1].startsWith("\"") && temp[1].endsWith("\""))
                    {
                        // String
                        objValue = temp[1].substring(1, temp[1].length() - 2); // Take off beginning and end quotes
                    }
                    else if(temp[1].equals(""))
                    {
                        // Empty String
                        objValue = temp[1];
                    }
                    else
                    {
                        // Double
                        objValue = Double.valueOf(temp[1]);
                    }

                    // Change corresponding value
                    keyMap.put(key, objValue);

                    if(objValue.getClass() == Boolean.class)
                    {
                        Robot.getTable().putBoolean(key, (Boolean)objValue);
                    }
                    else if(objValue.getClass() == String.class)
                    {
                        Robot.getTable().putString(key, (String)objValue);
                    }
                    else if(objValue.getClass() == Double.class)
                    {
                        Robot.getTable().putNumber(key, (Double)objValue);
                    }
                }
            }
            fr.close();
            
        } catch (FileNotFoundException ex) {
            try {
                // Create new file and add default values
                FileWriter fw = new FileWriter(DEFAULT_CSV_FILENAME);
                
                // Iterate through all keys to generate new default file
                Iterator i = keyMap.entrySet().iterator();
                while(i.hasNext())
                {
                    Map.Entry<String, Object> entry = (Map.Entry<String, Object>) i.next();
                    
                    // Write defaults depending on the data type
                    if(entry.getValue().getClass() == Boolean.class)
                    {
                        fw.write(entry.getKey() + ", " + Boolean.toString((Boolean)entry.getValue()) + s_lineSeparator);
                    }
                    else if(entry.getValue().getClass() == String.class)
                    {
                        fw.write(entry.getKey() + ", " + "\"" + entry.getValue() + "\"" + s_lineSeparator);
                    }
                    else if(entry.getValue().getClass() == Double.class)
                    {
                        fw.write(entry.getKey() + ", " + Double.toString((Double)entry.getValue()) + s_lineSeparator);
                    }
                    
                }
                
                fw.flush();
                fw.close();
                
                // Set NetworkTable values to default depending on data type
                for(String mapKey : keyMap.keySet())
                {
                    if(keyMap.get(mapKey).getClass() == Boolean.class)
                    {
                        if(keyMap.get(mapKey) != null)
                        {
                            Robot.getTable().putBoolean(mapKey, (Boolean)keyMap.get(mapKey));
                        }
                    }
                    else if(keyMap.get(mapKey).getClass() == String.class)
                    {
                        Robot.getTable().putString(mapKey, (String)keyMap.get(mapKey));
                    }
                    else if(keyMap.get(mapKey).getClass() == Double.class)
                    {
                        Robot.getTable().putNumber(mapKey, (Double)keyMap.get(mapKey));
                    }
                }
                
            } catch (IOException iEx) {
                handleCSVFileError(iEx);
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
                    
                    // Iterate through all keys and write current values to file
                    Iterator i = keyMap.entrySet().iterator();
                    while(i.hasNext())
                    {
                        Map.Entry<String, Object> entry = (Map.Entry<String, Object>) i.next();
                        
                        // Write values depending on the data type
                        if(entry.getValue().getClass() == Boolean.class)
                        {
                            fw.write(entry.getKey() + ", " + Boolean.toString((Boolean)entry.getValue()) + s_lineSeparator);
                        }
                        else if(entry.getValue().getClass() == String.class)
                        {
                            fw.write(entry.getKey() + ", " + "\"" + (String)entry.getValue() + "\"" + s_lineSeparator);
                        }
                        else if(entry.getValue().getClass() == Double.class)
                        {
                            fw.write(entry.getKey() + ", " + Double.toString((Double)entry.getValue()) + s_lineSeparator);
                        }
                    }
                    
                    fw.flush();
                    fw.close();
                } 
                catch (FileNotFoundException ex) 
                {
                    handleCSVFileError(ex);
                }
                catch (IOException iEx)
                {
                    handleCSVFileError(iEx);
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
        for(String mapKey : keyMap.keySet())
        {
            keyMap.put(mapKey, Robot.getTable().getNumber(mapKey));
            
            if(keyMap.get(mapKey).getClass() == Boolean.class)
            {
                if(keyMap.get(mapKey) != null)
                {
                    keyMap.put(mapKey, Robot.getTable().getBoolean(mapKey));
                }
            }
            else if(keyMap.get(mapKey).getClass() == String.class)
            {
                keyMap.put(mapKey, Robot.getTable().getString(mapKey));
            }
            else if(keyMap.get(mapKey).getClass() == Double.class)
            {
                keyMap.put(mapKey, Robot.getTable().getNumber(mapKey));
            }
        }
    }
    
    private void updateFromProperties() 
    {
        // PROBABLY THE RIGHT WAY TO DO IT
        // Comment this out to do offsite testing
        Robot.setHost(IP_PROPERTY.getValue());
        Robot.setPort(PORT_PROPERTY.getValue());
        Robot.setTeam(TEAM_NUMBER_PROPERTY.getValue());
        Robot.getTable().addTableListener(this);
        
        // Setup Network table (MUST BE BEFORE LOAD CSV LOGIC)
//        NetworkTable.setTeam(TEAM_NUMBER_PROPERTY.getValue());
//        NetworkTable.setIPAddress(IP_PROPERTY.getValue());
//        netTable = NetworkTable.getTable(TABLE_NAME_PROPERTY.getValue());
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
                heading = Robot.getTable().getNumber("Heading");
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
            linePt3 = new WPIPoint(0, size.height()/2);
            linePt4 = new WPIPoint(size.width(), size.height()/2);
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
        opencv_imgproc.cvThreshold(hue1, hue2, (Double)keyMap.get(hueMaxKey), 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        opencv_imgproc.cvThreshold(hue1, hue1, (Double)keyMap.get(hueMinKey), 255, opencv_imgproc.CV_THRESH_BINARY);

        // Saturation
        opencv_imgproc.cvThreshold(sat1, sat2, (Double)keyMap.get(satMaxKey), 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        opencv_imgproc.cvThreshold(sat1, sat1, (Double)keyMap.get(satMinKey), 255, opencv_imgproc.CV_THRESH_BINARY);

        // Value
        opencv_imgproc.cvThreshold(val1, val2, (Double)keyMap.get(valMaxKey), 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        opencv_imgproc.cvThreshold(val1, val1, (Double)keyMap.get(valMinKey), 255, opencv_imgproc.CV_THRESH_BINARY);

        
        // Combine the results to obtain our binary image which should for the most
        // part only contain pixels that we care about (Bin is initialized with cvAnd of hue1 and hue2
        // and then is bitwise anded incrementally with each new thresholded image
          opencv_core.cvAnd(hue1, hue2, bin, null);
          opencv_core.cvAnd(sat1, bin, bin, null);
          opencv_core.cvAnd(sat2, bin, bin, null);
          opencv_core.cvAnd(val1, bin, bin, null);
          opencv_core.cvAnd(val2, bin, bin, null);

        // Uncomment the next two lines to see the raw binary image
//        CanvasFrame result = new CanvasFrame("binary");
//        cf.showImage(bin.getBufferedImage());
        
        // Fill in any gaps using binary morphology
        // Changing the 5th parameter changes the method, and changing the 6th parameter changes the number of iterations
        // of the pixel extrapolation process.
        opencv_imgproc.cvMorphologyEx(bin, bin, null, morphKernel, opencv_imgproc.CV_MOP_CLOSE, kHoleClosingIterations);

        // Uncomment the next two lines to see the image post-morphology
        //CanvasFrame result2 = new CanvasFrame("morph");
//        cf.showImage(bin.getBufferedImage());

        // Find contours
        WPIBinaryImage binWpi = DaisyExtensions.makeWPIBinaryImage(bin);
        contours = DaisyExtensions.findConvexContours(binWpi);

        // Process image for rectangular goals
        this.processForRectangularGoals(rawImage, heading);

        // Draw a crosshair (line down the middle)
        rawImage.drawLine(linePt1, linePt2, alignedColor, 2);
        
        // Draw horizontal line in the middle
        rawImage.drawLine(linePt3, linePt4, alignedColor, 2);

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
    
    private void processForRectangularGoals(WPIColorImage rawImage, double heading) 
    {
        rectGoalPolygons = new ArrayList<WPIPolygon>();
        Double minWidthPixels = (Double)keyMap.get(minWidthRectGoalsKey);
        Double maxWidthPixels = (Double)keyMap.get(maxWidthRectGoalsKey);
        
        for (WPIContour c : contours)
        {
            double ratio = ((double) c.getHeight()) / ((double) c.getWidth());
            if (ratio < 0.5 && ratio > 0.05 && c.getWidth() > minWidthPixels && c.getWidth() < maxWidthPixels)
            {
                rectGoalPolygons.add(c.approxPolygon(20));
            }
        }
        
        WPIPolygon square = null;
        int highest = Integer.MAX_VALUE;

        for (WPIPolygon p : rectGoalPolygons)
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

            double topTargetHeightInches = (Double)keyMap.get(topTargetHeightInchesKey);
            double cameraHeightInches = (Double)keyMap.get(cameraHeightInchesKey);
            double cameraPitchDeg = (shooterTilted ? (Double)keyMap.get(cameraPitchDegHighKey) : (Double)keyMap.get(cameraPitchDegLowKey));
            
            // Find azimuth (horizontal degrees needed to line up with target). This is given as -180 being completely left,
            // +180 being completely right, and 0 being completely lined up.
            double azimuth = this.boundAngle0to180DegreesWithDirection(x*kHorizontalFOVDeg/2.0 + heading - kShooterOffsetDeg);
            double range = (topTargetHeightInches - cameraHeightInches)
                    / Math.tan((y*kVerticalFOVDeg/2.0 + cameraPitchDeg)*Math.PI/180.0);

            if (!m_debugMode)
            {
                // Outputs values to the dashboard
                Robot.getTable().putBoolean("found", true);
                Robot.getTable().putNumber("azimuth", azimuth);
                Robot.getTable().putNumber("range", range);
            } else
            {
                System.out.println("Target found");
                System.out.println("x: " + x);
                System.out.println("y: " + y);
                System.out.println("azimuth: " + azimuth);
                System.out.println("range: " + range);
            }
            
            // Draw outline around highest goal
            double goalAlignTolerance = (Double)keyMap.get(goalAlignToleranceKey);
            double centerX = square.getX() + square.getWidth()/2;
            double centerY = square.getY() + square.getHeight()/2;
            if(centerX >= linePt1.getX()-goalAlignTolerance && centerX <= linePt1.getX()+goalAlignTolerance
                    && centerY >= linePt3.getY()-goalAlignTolerance && centerY <= linePt3.getY()+goalAlignTolerance)
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
    }

    private void processForPyramidGoals(WPIColorImage rawImage, double heading)
    {
        // First we find the two rectangles, then we find the two closest
        // to each other, then we find the points to make a triangle, then
        // we find the centroid of the imaginary triangle.
        
        // THESE VALUES OF THE CONTOUR RATIOS ARE ASSUMED TO WORK BUT
        // ARE PROBABLY NOT THE RIGHT ONES SINCE THEY DO NOT REPRESENT
        // THE SMALL RECTANGULAR TARGETS
        
        // Categorize polygons as either horizontal or vertical pyramid polygons
        pyramidGoalHPolygons = new ArrayList<WPIPolygon>();
        pyramidGoalVPolygons = new ArrayList<WPIPolygon>();
        for (WPIContour c : contours)
        {
            double ratio = ((double) c.getHeight()) / ((double) c.getWidth());
            if (ratio < 1.0 && ratio > 0.5 && c.getWidth() > kMinWidthPyramidHGoals && c.getWidth() < kMaxWidthPyramidHGoals)
            {
                pyramidGoalHPolygons.add(c.approxPolygon(20));
            }
            else if(ratio < 1.0 && ratio > 0.5 && c.getWidth() > kMinWidthPyramidVGoals && c.getWidth() < kMaxWidthPyramidVGoals)
            {
                pyramidGoalVPolygons.add(c.approxPolygon(20));
            }
        }

        // Look for horizontal pyramid goal rects
        horizontalPyramidRects = new ArrayList<HorizontalPyramidRect>(); // Stores all valid horizontal pyramid rects and their values
        for (WPIPolygon p : pyramidGoalHPolygons)
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
                    // Draw a polygon overlay on the image (visual aid for drivers to see the rectangles)
                    rawImage.drawPolygon(p, WPIColor.BLUE, 2);

                    // Store the left, right, and midpoint coordinates for future calculations
                    // The Y values are the middle of the polygon, not the top or bottom
                    HorizontalPyramidRect rect = new HorizontalPyramidRect(p.getX(),                    //LeftX
                                                                            p.getX() + p.getWidth(),    //RightX
                                                                            p.getY() + p.getHeight()/2, //LeftY
                                                                            p.getY() + p.getHeight()/2, //RightY
                                                                            p.getWidth(), p.getHeight());   //Width, Height
                    
                    horizontalPyramidRects.add(rect); // add the rect to the arraylist of valid horizontal pyramid rects
                }
            }
            else
            {
                // Not an acceptable target
                rawImage.drawPolygon(p, WPIColor.YELLOW, 1);
            }
        }
        
        // Look for vertical pyramid goal rects
        verticalPyramidRects = new ArrayList<VerticalPyramidRect>(); // Stores all valid vertical pyramid rects and their values
        for (WPIPolygon p : pyramidGoalVPolygons)
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
                    // Draw a polygon overlay on the image (visual aid for drivers to see the rectangles)
                    rawImage.drawPolygon(p, WPIColor.BLUE, 2);

                    // Store the top point of the vertical polygon
                    // The X value is the middle of the polygon, and the Y value is the top of the polygon
                    VerticalPyramidRect rect = new VerticalPyramidRect(p.getX() + p.getWidth()/2,       //TopX
                                                                        p.getY(),                       //TopY
                                                                        p.getWidth(), p.getHeight());   //Width, Height
                    
                    verticalPyramidRects.add(rect); // add the rect to the arraylist of valid vertical pyramid rects
                }
            }
            else
            {
                // Not an acceptable target
                rawImage.drawPolygon(p, WPIColor.YELLOW, 1);
            }
        }
        
        // Find horizontal and vertical rects that are closest to each other in proportion to their size
        // The proportion makes it so that farther away goals that appear smaller in the image don't get
        // priority when looking for correct goals.
        
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
        
        return angle * -1; // Tweak
        
    }

    private void handleMiscError(Exception e)
    {
        e.printStackTrace();
        JOptionPane.showMessageDialog(null,
            "An error occurred in the program.",
            "Error in program",
            JOptionPane.ERROR_MESSAGE);
    }
    
    private void handleCSVFileError(Exception e)
    {
        e.printStackTrace();
        JOptionPane.showMessageDialog(null,
            "An error occurred when attempting to "
            + "open the CSV file. ",
            "Unable to Open CSV File",
            JOptionPane.ERROR_MESSAGE);
    }
    
   @Override
    public void valueChanged(ITable itable, String key, Object value, boolean newValue) 
    {
        if(key.equals(shooterTiltedKey)) // Update local tilt value
        {
            shooterTilted = (Boolean)value;
        }
        
        if(!newValue) // Make sure this is set to not pay attention to new values (ERRORS WILL HAPPEN AT loadSettingsFile)
        {
            // If key value was changed concerning this widget
            boolean concernsUs = false;
            for(String mapKey : keyMap.keySet())
            {
                // If key is needed by this program
                if(mapKey.equals(key))
                {
                    concernsUs = true;
                }
            }

            if(concernsUs)
            {
                // Reload the values into the widget
                this.updateLocalSettings();
            }
        }
    }

   /* Property changed occurs when one of the widget's properties is changed by
    * right-clicking the widget and hitting "properties." */
    @Override
    public void propertyChanged(Property property) 
    {
        if(property.equals(ipKey) || property.equals(portKey) || property.equals(teamNumberKey)
                || property.equals(tableNameKey))
        {
            this.updateFromProperties();
        }
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
