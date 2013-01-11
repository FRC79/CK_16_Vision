package basiccameraextensionwidget;

import com.googlecode.javacv.CanvasFrame;
import edu.wpi.first.smartdashboard.camera.WPICameraExtension;
import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIImage;
import java.io.File;
import java.io.IOException;
import java.util.Scanner;
import javax.imageio.ImageIO;

/**
 *
 * @author sebastian
 */
public class BasicCameraExtensionWidget extends WPICameraExtension{

	/* Here, we override the ordinary WPICameraExtension 
	 * processImage function to perform custom image processing.
	 * The rawImage parameter, which can either be of the type
	 * WPIColorImage or WPIGrayscaleImage, is the raw camera
	 * image coming in from the camera data sent to the dashboard.
	 * The WPIImage we return in this function is the output image
	 * that will appear on the dashboard. This allows us to project
	 * layers on top of images to the viewers of the dashboard.
	 */
	@Override
	public WPIImage processImage(WPIColorImage rawImage) {
		// Returns green values of the image
		return rawImage.getGreenChannel();
		
		// Returning the superclass's processImage function would return 
                // the regular camera image.
		//return super.processImage(rawImage);
	}
        
        public static void main(String[] args){
        if (args.length == 0)
        {
            System.out.println("Usage: Arguments are paths to image files to test the program on");
            return;
        }

        // Create the widget
        BasicCameraExtensionWidget widget = new BasicCameraExtensionWidget();

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

