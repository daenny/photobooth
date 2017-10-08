#!/usr/bin/env python
# Created by br _at_ re-web _dot_ eu, 2015-2016

import cProfile
import threading
import os
import pygame
from datetime import datetime
from glob import glob
from sys import exit
from time import sleep, time

from PIL import Image

from events import ArduinoSerial
from gui import GuiException, GUI_PyGame as GuiModule, check_for_event, wait_for_event
from camera import CameraException, Camera_folder as CameraModule
# from camera import CameraException, Camera_cv as CameraModule
# from camera import CameraException, Camera_gPhoto as CameraModule
from slideshow import Slideshow
from threading import Lock
# Printing depends on the optional python-cups module
try:
    import cups
except ImportError:
    cups = None
    print "cups module missing, so photo printing is disabled. To fix, please run:"
    print "sudo apt-get install python-cups"
import random
    
#####################
### Configuration ###
#####################

# Screen size (set to 0,0 to use native resolution)
display_size = (0, 0)
#display_size = (1024, 768)

# Is the monitor on its side? (For portrait photos on landscape monitors).
# If True, text will be rotated 90 degrees counterclockwise
display_rotate = False

# Is the camera on its side? (For portrait orientation. Note: EXIF is ignored!)
# If True, the "right" side of the photo will be assumed to be the actual top.
camera_rotate = False

# Final size of assembled image (the montage of four thumbnails).
# If printing, this should be same aspect ratio as the printer page.
# (E.g., 6x4 photo paper @392dpi == 2352x1568)
picture_size = (7 * 600, 5 * 600)

# Waiting time in seconds for posing
pose_time = 3

# Display time for assembled picture
display_time = 10

# Show a slideshow of existing pictures when idle
idle_slideshow = True

# Display time of pictures in the slideshow
slideshow_display_time = 5

# Default to sending every montage to the printer?
auto_print = False

# What filename for the shutter sound when taking pictures?
# Set to None to have no sound.
shutter_sound = "shutter.wav"
bip1_sound = "bip1.wav"
bip2_sound = "bip2.wav"

# Temp directory for storing pictures
if os.access("/dev/shm", os.W_OK):
    tmp_dir = "/dev/shm/"  # Don't abuse Raspberry Pi SD card, if possible
else:
    tmp_dir = "/tmp/"

word_list = [
    ["ameisenscheisse", "Anna"],
    ["Pindakaas", "Xander"],
    ["Cheese", "Alex"],
    ["Smile", "Alex"],
]

    
class PictureList:
    """A simple helper class.

    It provides the filenames for the assembled pictures and keeps count
    of taken and previously existing pictures.
    """

    def __init__(self, basename):
        """Initialize filenames to the given basename and search for
        existing files. Set the counter accordingly.
        """

        # Set basename and suffix
        self.basename = basename
        self.suffix = ".jpg"
        self.count_width = 5
        # Ensure directory exists
        dirname = os.path.dirname(self.basename)
        if not os.path.exists(dirname):
            os.makedirs(dirname)

        # Find existing files
        count_pattern = "[0-9]" * self.count_width
        pictures = glob(self.basename + count_pattern + self.suffix)

        # Get number of latest file
        if len(pictures) == 0:
            self.counter = 0
        else:
            pictures.sort()
            last_picture = pictures[-1]
            self.counter = int(last_picture[-(self.count_width + len(self.suffix)):-len(self.suffix)])

        # Print initial infos
        print("Info: Number of last existing file: " + str(self.counter))
        print("Info: Saving assembled pictures as: " + self.basename + "XXXXX.jpg")

    def get(self, count):
        return self.basename + str(count).zfill(self.count_width) + self.suffix

    def get_last(self):
        return self.get(self.counter)

    def get_next(self):
        self.counter += 1
        return self.get(self.counter)


class PrinterModule:
    """Encapsulate all the photobooth printing functionality.

    It allows the photobooth to enqueue a JPG to be printed on the
    locally connected printer.
    """

    c = None  # This module's connection to CUPS
    printer = None  # The default printer to print to
    options = {}  # Options for printing

    def __init__(self):
        """Initialize printer defaults
        """

        # If we don't have python-cups installed, don't do anything
        if not cups:
            print ("Notice: python-cups is not installed, so printing is disabled")
            print ("To fix, run: sudo apt-get install python-cups")
            return

        # Create the connection to the default CUPS server
        try:
            self.c = cups.Connection()
        except RuntimeError as e:
            self.c = None
            print ("Error: Could not connect to CUPS server for printing: " + repr(e))
            print ("To fix, run: sudo apt-get install cups")
            return

        # Discover available printers 
        # Use default destination, if no queue is already defined
        if not self.printer:
            self.printer = self.c.getDefault()
        if not self.printer:
            # NO DEFAULT PRINTER! Let the user know how to fix it.
            if not self.c.getDests():
                print "Error: CUPS is running, but no printers are setup yet."
                print "To fix this error, first run: sudo addgroup $USER lpadmin"
                print "then, go to http://localhost:631/admin"
                return
            else:
                print "AVAILABLE PRINTERS"
                print self.c.getDests()
                print "Error: CUPS is running and at least one printer exists, but there is no server default printer."
                print "To fix this error: go to http://localhost:631/printers"
                return

        print "Printing enabled to: " + self.printer

        # Set default printing options
        if not self.options:
            self.options = {}

    def can_print(self):
        "Return True if printing is possible (CUPS running and default printer exists)"
        return self.c and self.printer

    def enqueue(self, filename):
        "Send a JPEG file to the printer using CUPS."
        if self.can_print():

            print "Now printing file " + filename + " to printer " + self.printer + ", using options " + repr(
                self.options)
            try:
                self.c.printFile(self.printer, filename, filename, self.options)
            except cups.IPPError as e:
                print ("Error: Failed to print " + filename + ": " + repr(e))


class Photobooth:
    """The main class.

    It contains all the logic for the photobooth.
    """

    def __init__(self, display_size, display_rotate, picture_size, pose_time, display_time,
                  idle_slideshow, slideshow_display_time):
        self.display = GuiModule('Photobooth', display_size)
        if display_size == (0, 0):  # Detect actual resolution
            display_size = self.display.get_size()
        self.display_rotate = display_rotate
        if display_rotate:
            self.display.set_rotate(True)
        # Image basename
        picture_basename = datetime.now().strftime("%Y-%m-%d/pic")
        assembled_picture_basename = datetime.now().strftime("%Y-%m-%d/assembled/pic")

        self.pictures = PictureList(picture_basename)
        self.assembled_pictures = PictureList(assembled_picture_basename)
        self.taking_picture = Lock()
        self.arduino = ArduinoSerial()
        self.camera = CameraModule((picture_size[0] / 2, picture_size[1] / 2), camera_rotate=camera_rotate)
        self.camera_rotate = camera_rotate

        self.picture_size = picture_size
        self.pose_time = pose_time
        self.display_time = display_time

        self.idle_slideshow = idle_slideshow
        if self.idle_slideshow:
            self.slideshow_display_time = slideshow_display_time
            self.slideshow = Slideshow(display_size, display_time,
                                       os.path.dirname(os.path.realpath(picture_basename)),
                                       recursive=False)
            if display_rotate:
                self.slideshow.display.set_rotate(True)

        self.printer_module = PrinterModule()
        try:
            pygame.mixer.init(buffer=1024)
            self.shutter = pygame.mixer.Sound(shutter_sound)
            self.bip1 = pygame.mixer.Sound(bip1_sound)
            self.bip2 = pygame.mixer.Sound(bip2_sound)
            self.bip2.play()
        except pygame.error:
            self.shutter = None
            pass
        self.arduino.start()

    def teardown(self):
        self.arduino.stop()
        self.arduino.join()
        self.display.msg("Shutting down...")
        self.display.teardown()
        self.remove_tempfiles()
        sleep(0.5)

        exit(0)

    def remove_tempfiles(self):
        for filename in glob(tmp_dir + "photobooth_*.jpg"):
            try:
                os.remove(filename)
            except OSError:
                pass

    def _run_plain(self):
        while True:
            self.camera.set_idle()

            # Display default message
            self.display.msg("Hit the button!")

            # Wait for an event and handle it
            event = wait_for_event()
            self.handle_event(event)

    def _run_slideshow(self):
        while True:
            self.camera.set_idle()
            self.slideshow.display_next("Hit the button!")
            tic = time()
            while time() - tic < self.slideshow_display_time:
                self.check_and_handle_events()
                sleep(0.1)

    def run(self):
        while True:
            try:
                # Select idle screen type
                if self.idle_slideshow:
                    self._run_slideshow()
                else:
                    self._run_plain()

            # Catch exceptions and display message
            except CameraException as e:
                self.handle_exception(e.message)
            # Do not catch KeyboardInterrupt and SystemExit
            except (KeyboardInterrupt, SystemExit):
                raise
            except Exception as e:
                import sys
                print('SERIOUS ERROR' + repr(e))
                sys.excepthook(*sys.exc_info())
                self.handle_exception("SERIOUS ERROR!\n(see log file)")

    def check_and_handle_events(self):
        r, e = check_for_event()
        while r:
            self.handle_event(e)
            r, e = check_for_event()

    def handle_serial_command(self, event):
        with self.taking_picture:
            self.take_picture()

    def clear_event_queue(self):
        r, e = check_for_event()
        while r:
            r, e = check_for_event()

    def handle_event(self, event):
        if event.type == 0:
            self.teardown()
        if self.taking_picture.locked():
            return
        if event.type == 1:
            self.handle_keypress(event.value)
        elif event.type == 2:
            self.handle_mousebutton(event.value[0], event.value[1])
        elif event.type == 3:
            self.handle_serial_command(event.value)

    def handle_keypress(self, key):
        """Implements the actions for the different keypress events"""
        # Exit the application
        if key == ord('q'):
            self.teardown()
        # Take pictures
        elif key == ord('c'):
            with self.taking_picture:
                self.take_picture()
        elif key == ord('f'):
            print("ToggleScreen")
            self.display.toggle_fullscreen()
        elif key == ord('i'):  # Re-initialize the camera for debugging
            self.camera.reinit()
        elif key == ord('p'):
            self.toggle_auto_print()
        elif key == ord('r'):
            self.toggle_rotate()

    def toggle_auto_print(self):
        "Toggle auto print and show an error message if printing isn't possible."
        if self.printer_module.can_print():
            global auto_print
            auto_print = not auto_print
            self.display.msg("Autoprinting %s" % ("enabled" if auto_print else "disabled"))
        else:
            self.display.msg("Printing not configured\n(see log file)")

    def toggle_rotate(self):
        "Toggle rotating the display and camera."
        self.toggle_display_rotate()
        self.toggle_camera_rotate()
        self.display.msg("Display and camera rotated")

    def toggle_display_rotate(self):
        "Toggle rotating the display 90 degrees counter clockwise."
        self.display_rotate = (not self.display_rotate)
        self.display.set_rotate(self.display_rotate)
        self.slideshow.display.set_rotate(self.display_rotate)

    def toggle_camera_rotate(self):
        "Toggle rotating the camera 90 degrees counter clockwise."
        self.camera_rotate = (not self.camera_rotate)
        self.camera.set_rotate(self.camera_rotate)

    def handle_mousebutton(self, key, pos):
        """Implements the actions for the different mousebutton events"""
        # Take a picture
        if key == 1:
            with self.taking_picture:
                self.take_picture()

    def handle_exception(self, msg):
        """Displays an error message and returns"""
        print("Error: " + msg)
        try:
            self.display.msg("ERROR:\n\n" + msg)
        except GuiException:
            self.display.msg("ERROR")
        sleep(3)

    def assemble_picture(self, input_filename):
        (H, W) = self.picture_size

        # Thumbnail size of pictures
        inner_border = 0
        thumb_box = (int(W),
                     int(H / 2))
        thumb_size = (thumb_box[0],
                      thumb_box[1] - inner_border/2)

        # Create output image with white background
        output_image = Image.new('RGB', (W, H), (255, 255, 255))

        # Image 0
        img = Image.open(input_filename)
        img = img.resize(maxpect(img.size, thumb_size), Image.ANTIALIAS)

        crop = int((img.size[1] - thumb_size[1])/2)

        img = img.crop((0, crop, img.size[0], img.size[1]-crop))

        offset = (0, 0)
        output_image.paste(img, offset)

        # Image copy
        offset = (0,
                  thumb_box[1] + inner_border)
        output_image.paste(img, offset)

        # Save assembled image
        ass_output_filename = self.assembled_pictures.get_next()
        output_image.save(ass_output_filename, "JPEG", quality=95)

        # Save Input Image
        output_filename = self.pictures.get_next()
        img.save(output_filename, "JPEG", quality=95)

        return ass_output_filename, output_filename

    def assemble_pictures(self, input_filenames):
        """Assembles four pictures into a 2x2 grid of thumbnails.

        The total size (WxH) is assigned in the global variable
        assembled_size at the top of this file. (E.g., 2352x1568)

        The outer border (a) is 2% of W
        The inner border (b) is 1% of W

        Note that if the camera is on its side, H and W will be
        swapped to create a portrait, rather than landscape, montage.

        Thumbnail sizes are calculated like so:
        h = (H - 2 * a - 2 * b) / 2
        w = (W - 2 * a - 2 * b) / 2

                                    W
               |---------------------------------------|

          ---  +---+-------------+---+-------------+---+  ---
           |   |                                       |   |  a
           |   |   +-------------+   +-------------+   |  ---
           |   |   |             |   |             |   |   |
           |   |   |      0      |   |      1      |   |   |  h
           |   |   |             |   |             |   |   |
           |   |   +-------------+   +-------------+   |  ---
         H |   |                                       |   |  2*b
           |   |   +-------------+   +-------------+   |  ---
           |   |   |             |   |             |   |   |
           |   |   |      2      |   |      3      |   |   |  h
           |   |   |             |   |             |   |   |
           |   |   +-------------+   +-------------+   |  ---
           |   |                                       |   |  a
          ---  +---+-------------+---+-------------+---+  ---

               |---|-------------|---|-------------|---|
                 a        w       2*b       w        a

        [Note that extra padding will be added on the sides if the
        aspect ratio of the camera images do not match the aspect
        ratio of the final assembled image.]

        """

        # If the display is in portrait orientation,
        # we should create an assembled image that fits it. 
        if self.display.get_rotate():
            (H, W) = self.picture_size
        else:
            (W, H) = self.picture_size

        # Thumbnail size of pictures
        outer_border = int(2 * max(W, H) / 100)  # 2% of long edge
        inner_border = int(1 * max(W, H) / 100)  # 1% of long edge
        thumb_box = (int(W / 2),
                     int(H / 2))
        thumb_size = (thumb_box[0] - outer_border - inner_border,
                      thumb_box[1] - outer_border - inner_border)

        # Create output image with white background
        output_image = Image.new('RGB', (W, H), (255, 255, 255))

        # Image 0
        img = Image.open(input_filenames[0])
        img = img.resize(maxpect(img.size, thumb_size), Image.ANTIALIAS)
        offset = (thumb_box[0] - inner_border - img.size[0],
                  thumb_box[1] - inner_border - img.size[1])
        output_image.paste(img, offset)

        # Image 1
        img = Image.open(input_filenames[1])
        img = img.resize(maxpect(img.size, thumb_size), Image.ANTIALIAS)
        offset = (thumb_box[0] + inner_border,
                  thumb_box[1] - inner_border - img.size[1])
        output_image.paste(img, offset)

        # Image 2
        img = Image.open(input_filenames[2])
        img = img.resize(maxpect(img.size, thumb_size), Image.ANTIALIAS)
        offset = (thumb_box[0] - inner_border - img.size[0],
                  thumb_box[1] + inner_border)
        output_image.paste(img, offset)

        # Image 3
        img = Image.open(input_filenames[3])
        img = img.resize(maxpect(img.size, thumb_size), Image.ANTIALIAS)
        offset = (thumb_box[0] + inner_border,
                  thumb_box[1] + inner_border)
        output_image.paste(img, offset)

        # Save assembled image
        output_filename = self.pictures.get_next()
        output_image.save(output_filename, "JPEG")
        return output_filename

    def show_preview(self, message=""):
        """If camera allows previews, take a photo and show it so people can
        pose before the shot. For speed, previews are decimated to fit
        within the screen instead of being scaled. For even more
        speed, the previews are blitted directly to a subsurface of
        the display. (Converting to a pygame Surface is slower). 

        """
        self.display.clear()
        if self.camera.has_preview():
            f = self.camera.get_preview_array(self.display.get_size())
            self.display.blit_array(f)
        self.display.show_message(message)
        self.display.apply()

    def show_counter(self, seconds):
        """Loop over showing the preview (if possible), with a count down"""
        tic = time()
        toc = time() - tic
        old_t = None
        while toc < seconds:
            t = seconds - int(toc)
            if t != old_t and self.bip1:
                self.display.msg(str(t))
                self.bip1.play()
                self.say("{}".format(t))
            old_t = t
            self.display.msg(str(t))
            
            # Limit progress to 1 "second" per preview (e.g., too slow on Raspi 1)
            toc = min(toc + 1, time() - tic)

    def show_pose(self, seconds, message=""):
        """Loop over showing the preview (if possible), with a static message.

        Note that this is *necessary* for OpenCV webcams as V4L will ramp the
        brightness level only after a certain number of frames have been taken.
        """

        tic = time()
        toc = time() - tic
        while toc < seconds:
            self.show_preview(message)
            # Limit progress to 1 "second" per preview (e.g., too slow on Raspi 1)
            toc = min(toc + 1, time() - tic)

    def say(self, text, voice="Alex"):
        os.system("say -v {} -r 200 '{}'".format(voice, text))

    def say_with_delay(self,text,voice="Alex", delay=0.2):
        sleep(delay)
        self.say(text,voice)
        
    def take_picture(self):
        """Implements the picture taking routine"""
        # NUM PICs
        num_pics = 1
        t = threading.Thread(target=self.say, args=("Pose for the photo!", ))
        t.start()
        
        activate_olympus = """
        osascript -e 'tell application "OLYMPUS Capture" to activate'
        """
        os.system(activate_olympus)
        self.display.msg("\n\n\n\n\n\n\nTaking {} picture ...".format(num_pics))
        sleep(3.0)
        t.join()
        # Extract display and image sizes
        display_size = self.display.get_size()

        focus_python()
        self.display.msg("Look at the camera...".format(num_pics))
        self.say("Look at the camera")
        
        sleep(1.0)
        words = random.choice(word_list)
        words.append(0.3)
        # Take pictures
        filenames = [i for i in range(num_pics)]
        for x in range(num_pics):
            # Countdown
            self.show_counter(self.pose_time)
            self.display.msg("")
            #self.display.msg("S M I L E !!!\n\n {} of {}".format(x + 1, num_pics))
            
            t = threading.Thread(target=self.say_with_delay, args=tuple(words))
            t.start()
            try:
                #t = threading.Thread(target=self.camera.take_picture, args=(tmp_dir + "photobooth_%02d.jpg" % x, ))
                #t.start()

                filenames[x] = self.camera.take_picture(tmp_dir + "photobooth_%02d.jpg" % x)
                #if self.shutter:
                #    self.shutter.play()

            except CameraException as e:
                raise e
            t.join()
        # Show 'Wait'
        self.display.msg("Please wait!\n\nWorking\n...")

        # Assemble them
        ass_outfile, outfile = self.assemble_picture(filenames[0])

        if self.printer_module.can_print():
            # Show picture for 10 seconds and then send it to the printer.
            # If auto_print is True,  hitting the button cancels the print.
            # If auto_print is False, hitting the button sends the print
            tic = time()
            t = int(self.display_time - (time() - tic))
            old_t = self.display_time + 1
            do_print = auto_print
            # Clear event queue (in case they hit the button twice accidentally) 
            self.clear_event_queue()

            while t > 0:
                if t != old_t:
                    self.display.clear()
                    self.display.show_picture(outfile, display_size, (0, 0))
                    self.display.show_message("%s%d" % ("Wait for print \n or press button to cancel!\n " if auto_print else "Press button to print photo!\n", t))
                    self.display.apply()
                    old_t = t

                # Watch for button, gpio, mouse press to cancel/enable printing
                r, e = check_for_event()
                if r:  # Caught a button press.
                    self.display.clear()
                    self.display.show_picture(outfile, display_size, (0, 0))
                    self.display.show_message("Printing%s" % (" cancelled" if auto_print else ""))
                    self.display.apply()
                    self.clear_event_queue()
                    # Discard extra events (e.g., they hit the button a bunch)
                    sleep(2)
                    self.clear_event_queue()
                    do_print = not do_print

                    break

                t = int(self.display_time - (time() - tic))

            # Either button pressed or countdown timed out
            if do_print:
                self.display.msg("Printing")
                self.printer_module.enqueue(ass_outfile)

        else:
            # No printer available, so just show montage for 10 seconds
            self.display.clear()
            self.display.show_picture(outfile, display_size, (0, 0))
            self.display.apply()
            sleep(self.display_time)


#################
### Functions ###
#################

def maxpect(a, b):
    '''Given two width by height sizes a and b, return the maximum WxH
    that is the same aspect ratio as a and fits within b.
    '''
    w_ratio = float(b[0]) / a[0]
    h_ratio = float(b[1]) / a[1]
    ratio = max(w_ratio, h_ratio)

    return (int(ratio * a[0]), int(ratio * a[1]))


def focus_python():
    activate_python = """
    osascript -e 'tell application "Python" to activate'
    """


    print "Activating Python"
    t = threading.Thread(target=os.system, args=(activate_python, ))
    t.start()
    for i in range(10):
        check_for_event()
        sleep(0.1)
    t.join()
    print "done"

pr = None


def begin_profile():
    "Run this before entering a slow part of the program"
    global pr
    pr = cProfile.Profile()
    pr.enable()


def end_profile():
    "Run this after exiting a slow part of the program"
    global pr
    pr.disable()
    import StringIO
    s = StringIO.StringIO()
    sortby = 'time'
    import pstats
    ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
    ps.print_stats()
    print s.getvalue()


def main():
    photobooth = Photobooth(display_size, display_rotate,
                            picture_size, pose_time, display_time,
                            idle_slideshow, slideshow_display_time)
    photobooth.clear_event_queue()  # Flush button presses
    photobooth.run()
    photobooth.teardown()
    return 0


if __name__ == "__main__":
    exit(main())
