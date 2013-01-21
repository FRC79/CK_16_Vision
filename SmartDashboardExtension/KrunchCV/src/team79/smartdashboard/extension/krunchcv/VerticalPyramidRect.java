
package team79.smartdashboard.extension.krunchcv;

/**
 *
 * @author sebastian
 */
public class VerticalPyramidRect {
    
    double topPointX, topPointY, width, height;
    
    public VerticalPyramidRect(double topPointX, double topPointY, 
            double width, double height)
    {
        this.topPointX = topPointX;
        this.topPointY = topPointY;
        this.width = width;
        this.height = height;
    }

    public double getTopPointX() {
        return topPointX;
    }

    public void setTopPointX(double topPointX) {
        this.topPointX = topPointX;
    }

    public double getTopPointY() {
        return topPointY;
    }

    public void setTopPointY(double topPointY) {
        this.topPointY = topPointY;
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
