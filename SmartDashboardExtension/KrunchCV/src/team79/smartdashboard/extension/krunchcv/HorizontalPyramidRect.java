
package team79.smartdashboard.extension.krunchcv;

/**
 *
 * @author sebastian
 */
public class HorizontalPyramidRect {
    
    double leftX, leftY, rightX, rightY, width, height, midpointX, midpointY;
    
    public HorizontalPyramidRect(double leftX, double rightX, double leftY, double rightY,
            double width, double height)
    {
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.rightY = rightY;
        this.width = width;
        this.height = height;
        this.midpointX = calculateMidpointX(leftX, rightX);
        this.midpointY = calculateMidpointY(leftY, rightY);
    }
    
    private double calculateMidpointX(double p1X, double p2X)
    {
        return (p1X + p2X)/2;
    }
    
    private double calculateMidpointY(double p1Y, double p2Y)
    {
        return (p1Y + p2Y)/2;
    }

    public double getLeftX() {
        return leftX;
    }

    public void setLeftX(double leftX) {
        this.leftX = leftX;
    }

    public double getLeftY() {
        return leftY;
    }

    public void setLeftY(double leftY) {
        this.leftY = leftY;
    }

    public double getRightX() {
        return rightX;
    }

    public void setRightX(double rightX) {
        this.rightX = rightX;
    }

    public double getRightY() {
        return rightY;
    }

    public void setRightY(double rightY) {
        this.rightY = rightY;
    }

    public double getMidpointX() {
        return midpointX;
    }

    public void setMidpointX(double midpointX) {
        this.midpointX = midpointX;
    }

    public double getMidpointY() {
        return midpointY;
    }

    public void setMidpointY(double midpointY) {
        this.midpointY = midpointY;
    }

    public double getWidth() {
        return width;
    }

    public void setWidth(double width) {
        this.width = width;
    }

    public double getHeight() {
        return height;
    }

    public void setHeight(double height) {
        this.height = height;
    }

    
}
