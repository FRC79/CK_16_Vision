
package team79.smartdashboard.extension.krunchcv;

import edu.wpi.first.smartdashboard.gui.StaticWidget;
import edu.wpi.first.smartdashboard.properties.Property;
import edu.wpi.first.smartdashboard.robot.Robot;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.JButton;

/**
 *
 * @author sebastian
 */
public class SaveButton extends StaticWidget implements ActionListener{

    JButton saveButton;
    
    @Override
    public void init() {
        
        Robot.getPreferences().putBoolean("save", false); // Default to false
        
        saveButton = new JButton("Save Settings");
        saveButton.addActionListener(this);
        
        this.add(saveButton);
    }

    @Override
    public void propertyChanged(Property prprt) {
        
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        Robot.getPreferences().putBoolean("save", true);
    }
    
}
