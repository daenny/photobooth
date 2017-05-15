#!/usr/bin/env python
# Created by br@re-web.eu, 2015

import subprocess
import pygame
import numpy

cv_enabled = False
gphoto2cffi_enabled = False
piggyphoto_enabled = False

try:
    import cv2 as cv
    cv_enabled = True
    print('OpenCV available')
except ImportError:
    pass

try:
    import gphoto2cffi as gp
    gpExcept = gp.errors.GPhoto2Error
    gphoto2cffi_enabled = True
    print('Gphoto2cffi available')
except ImportError:
    pass

if not gphoto2cffi_enabled:
    try:
        import piggyphoto as gp
        gpExcept = gp.libgphoto2error
        piggyphoto_enabled = True
        print('Piggyphoto available')
    except ImportError:
        pass

class CameraException(Exception):
    """Custom exception class to handle camera class errors"""
    def __init__(self, message, recoverable=False):
        self.message = message
        self.recoverable = recoverable


class Camera_cv:
    def __init__(self, picture_size):
        global cv_enabled
        if cv_enabled:
            self.cap = cv.VideoCapture(-1)
            if not self.cap.isOpened:
                print "Warning: Failed to open camera using OpenCV"
                cv_enabled=False
                return
            self.cap.set(3, picture_size[0])
            self.cap.set(4, picture_size[1])

            # Warm up web cam for quick start later and to double check driver
            r, dummy = self.cap.read()
            if not r:
                print "Warning: Failed to read from camera using OpenCV"
                cv_enabled=False
                return

    def has_preview(self):
        return True 

    def take_preview(self, filename="/tmp/preview.jpg"):
        self.take_picture(filename)

    def get_preview_array(self, max_size=None):
        """Get a quick preview from the camera and return it as a 2D array
        suitable for quick display using pygame.surfarray.blit_array().

        If a maximum size -- (w,h) -- is passed in, the returned image
        will be quickly decimated using numpy to be at most that large.
        """

        # Grab a camera frame
        r, f = self.cap.read()

        # Optionally reduce frame size by decimation (nearest neighbor)
        if max_size:
            (max_w, max_h) = map(int, max_size)
            (    h,     w) = ( len(f), len(f[0]) ) # Note OpenCV swaps rows and columns
            w_factor = (w/max_w) + (1 if (w%max_w) else 0)
            h_factor = (h/max_h) + (1 if (h%max_h) else 0)
            scaling_factor = max( (w_factor, h_factor) )
            f=f[::scaling_factor, ::scaling_factor]

        # Convert from OpenCV format to Surfarray
        f=cv.cvtColor(f,cv.COLOR_BGR2RGB)
        f=numpy.rot90(f)
        return f

    def get_preview_pygame_surface(self, max_size=None):
        """Get a quick preview from the camera and return it as a Pygame
        Surface suitable for transformation and display using GUI.py's
        surface_list.
        """
        f = self.get_preview_array(max_size)
        ( w,  h) = ( len(f), len(f[0]) )

        # For some reason make_surface() is slower on an iMac than
        # creating a new surface and blitting the image to it. Weird!
        # I think this is the opposite for the Raspberry Pi 3b.
        if False:
            s=pygame.surfarray.make_surface(f)
        else:
            s = pygame.Surface((w,h))
            pygame.surfarray.blit_array(s, f)

        return s


    def take_picture(self, filename="/tmp/picture.jpg"):
        if cv_enabled:
            r, frame = self.cap.read()
            cv.imwrite(filename, frame)
            return filename
        else:
            raise CameraException("OpenCV not available!")

    def set_idle(self):
        pass


class Camera_gPhoto:
    """Camera class providing functionality to take pictures using gPhoto 2"""

    def __init__(self, picture_size):
        self.picture_size = picture_size
        # Print the capabilities of the connected camera
        try:
            if gphoto2cffi_enabled:
                self.cap = gp.Camera()
            elif piggyphoto_enabled:
                self.cap = gp.camera()
                print(self.cap.abilities)
            else:
                print(self.call_gphoto("-a", "/dev/null"))
        except CameraException as e:
            print('Warning: Listing camera capabilities failed (' + e.message + ')')
        except gpExcept as e:
            print('Warning: Listing camera capabilities failed (' + e.message + ')')

    def call_gphoto(self, action, filename):
        # Try to run the command
        try:
            cmd = "gphoto2 --force-overwrite --quiet " + action + " --filename " + filename
            output = subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT)
            if "ERROR" in output:
                raise subprocess.CalledProcessError(returncode=0, cmd=cmd, output=output)
        except subprocess.CalledProcessError as e:
            if "EOS Capture failed: 2019" in e.output or "Perhaps no focus" in e.output:
                raise CameraException("Can't focus!\nMove a little bit!", True)
            elif "No camera found" in e.output:
                raise CameraException("No (supported) camera detected!", False)
            elif "command not found" in e.output:
                raise CameraException("gPhoto2 not found!", False)
            else:
                raise CameraException("Unknown error!\n" + '\n'.join(e.output.split('\n')[1:3]), False)
        return output

    def has_preview(self):
        return gphoto2cffi_enabled or piggyphoto_enabled

    def take_preview(self, filename="/tmp/preview.jpg"):
        if gphoto2cffi_enabled:
            self._save_picture(filename, self.cap.get_preview())
        elif piggyphoto_enabled:
            self.cap.capture_preview(filename)	
        else:
            raise CameraException("No preview supported!")

    def take_picture(self, filename="/tmp/picture.jpg"):
        if gphoto2cffi_enabled:
            self._save_picture(filename, self.cap.capture())
        elif piggyphoto_enabled:
            self.cap.capture_image(filename)
        else:
            self.call_gphoto("--capture-image-and-download", filename)
        return filename

    def _save_picture(self, filename, data):
        f = open(filename, 'wb')
        f.write(data)
        f.close()

    def set_idle(self):
        if gphoto2cffi_enabled:
            self.cap._get_config()['actions']['viewfinder'].set(False)
        elif piggyphoto_enabled:
            # This doesn't work...
            self.cap.config.main.actions.viewfinder.value = 0
