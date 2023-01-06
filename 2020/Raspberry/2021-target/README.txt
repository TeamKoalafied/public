The following instructions assume that:
- FRCVision (eg https://github.com/wpilibsuite/WPILibPi/releases/tag/v2020.2.1) has
  been installed on the Raspberry Pi. 2021 introduced new versions of this, and
  renamed it to WPILibPi, but this has not been tested. Only the "base" image of
  this new variant seems to be needed.
- The Raspberry Pi has an IP address of 10.69.96.6 and is connected to the robot
  wifi radio
- Your laptop is connected to the robot network either directly, or via wifi
- You opened 10.69.96.6 in your web browser and can see the FRCVision page
- You have an ssh or PuTTY session open at 10.69.96.6 (usr: pi, pwd: raspberry)

How to build and install this code:
- In the web browser, set the system to Writeable with the radio button at the top
  of the main page.
- In the ssh/PuTTY session, create a directory /home/examples/lines if it does not
  exist, then cd to it.
    pi> mkdir /home/examples/lines
    pi> cd /home/examples/lines
- Copy the contents of this directory to a directory on the Raspberry Pi using
    laptop> put.bat * examples/lines
- In the ssh/PuTTY session, build and install the code using
    pi> make clean
    pi> make install
- In the web browser, set the system back to ReadOnly using the radio button at the
  top of the main page.

The code is now running, and will send the following data via network table "pivision" to the RoboRio:
  Target data:
    tx: the angle the robot needs to turn in degrees to align with the target.         Positive to the right.
    ty: the distance to the target in metres
    tv: 1.0 if the target is detected. Otherwise 0.0.
    Note that the stream at http://10.69.96.6:1183/stream.mjpg shows a green
    targeting sight when the target has been detected. This turns red if tx is
    between -0.5 and +0.5 degrees, indicating target alignment.
  Camera calibration:
    The cx and cy values adjust the cross-hairs that tx and ty are referenced to
    If the camera is moved, or the robot configuration changed, these settings can
    be adjusted from the dashboard 
    cx: Camera calibration horizontal crosshair position: Align this with the
        target when the shooter is aligned. It is better to set it to 160, and move
        the camera until the vertical cross-hair is aligned with the target, but
        you can change cx during a match if the camera gets knocked and the shooter
        is not aligning properly.
    cy: Camera calibration vertical crosshair position: This should be aligned with
        a marker placed at the same height as the camera several metres away. It
        is better to set it to 120 and move the camera until the horizontal
        cross-hair is aligned with the marker, but you can change cy during a match
        if the camera gets knocked and the distance readings are incorrect. 

Several video streams are available. These only consume bandwidth if you display them on your browser.
    http://10.69.96.6:1181/stream.mjpg - Front raw? [not used]
    http://10.69.96.6:1182/stream.mjpg - Back - Use this view for seeing what the
      intake is doing
    http://10.69.96.6:1183/stream.mjpg - Front - Use this view for aligning with
      the target with cross-hairs
    http://10.69.96.6:1184/stream.mjpg - Debug - this is what the pi is using to
      compute the angle and range. If there are too many detected lines, it will
      give incorrect results. If there are no detected lines, then tv will be 0.0
      and tx and ty will not be sent. You may need to darken the camera brightness
      to get good results in a bright room.
    http://10.69.96.6:1185/stream.mjpg - FrontZoom - this is a scaled up version of
      the centre of the Front view. However it is not particularly useful.

The .htm files in this directory can be used to view these streams on your browser.
They can be used while the robot is running, even with the field restrictions on
bandwidth.

Further notes:
- The GripPipeline processes video images to find lines in the image. You can use
  Grip to create new code for this pipeline if desired (eg to detect other features
  in the image). Grip is available here https://wpiroboticsprojects.github.io/GRIP.
