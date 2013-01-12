
Smart Dashboard Extensions 
for Client Vision Processing
---------------------------------------------------------------------

KrunchCV is based off of 
team 341's vision processing widget and an example 
from a WPILibCookbook.


http://www.wbrobotics.com/attachments/article/11/WPILibCookbook.pdf
This is the URL of the cookbook. While there are
useful things according to the SmartDashboard on
pages 56-79, the majority of this extension is just
on pages 72-75.

http://www.chiefdelphi.com/media/papers/2676
The source code for team 341's widget is at the URL above
in a zip file.

http://code.google.com/p/frc2012-team3929/wiki/VisionCoding
Team 3929's wiki that may help with this topic.

http://firstforge.wpi.edu/sf/frs/do/viewRelease/projects.smartdashboard/frs.installer.installer_1_0_3
This is the download link for the SmartDashboard, which is
necessary for this setup. It is an .exe, so Mac and Linux users
should just download and extract the SmartDashboard.zip from this
repository and run the SmartDashboard.jar.

Refer to the team 341 link for more details about the install.

The SmartDashboard has widgets that can be added by clicking
View->Add on the top menu. They can be edited by hitting 
View->Editable. You can then drag widgets around and right-click
them to change their properties. These widgets can be declared in
the robot's code to perform data output and functions. This is covered
in more detail in the cookbook.

To allow vision processing, it is necessary to extend the WPICameraExtension
with custom java code. The code currently implemented is in the repository named
KrunchCV. To add the widget to the SmartDashboard,
download KrunchCV either as a zip file or via Github
cloning, navigate inside the dist folder, copy the KrunchCV
jar file, navigate to where the SmartDashboard is installed (this is
usually C:\Program Files\SmartDashboard\ in Windows), navigate to the
extensions folder, and paste the jar file in there. Finally, start the
dashboard, go up to the top menu, and click View->Add->
Krunch Target Tracker


This code was compiled in Netbeans, however, it could be possible to compile in Eclipse.

To create an extension project, create a new java project in Netbeans. Then, add the
following libraries:

SmartDashboard.jar
WPICameraExtension.jar
javacpp.jar
javacv.jar
javacv-windows-x86.jar
WPIJavaCV.jar

These can be found in the install location of the SmartDashboard. The first is in the root
directory, the second is in the extensions directory, and the rest are in the extensions\lib
directory.

The javacv is a wrapper for the OpenCV library.

For the code to work, the main class should be put in a package starting with "edu".

Following the cookbook on page 72, override the processImage method after making a new class.
The class implemented also has a main method so it can run as a standalone program for testing. 
Make sure all imports are in order, and Build and Clean the project. If everything worked, a dist folder 
should appear in the project folder in the Netbeans workspace (where it is stored on the filesystem). 
Copy the jar file from the dist folder and place it into the SmartDashboard's extensions
folder to add it to the dashboard.



