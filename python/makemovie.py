#!/usr/bin/python


#
#
#
#  This is a script to convert ppm files to
#  jpg files, then a movie, then clean up the
#  jpg files.  Assuming you are created a 
#  collection of jpg files with names
#
#      some_base_name.XXXX.ppm
#
#  where XXXX is a zero-padded four digit frame
#  number, you can use this tool like this:
#
#       python ./makemovie.py some_base_name.*.ppm
#
#  when done, a file "test.mov" will be generated.
#
#  The ImageMagick routine "convert" is used in this script.
#
#





import os
import sys

if len(sys.argv) > 1:
    if sys.argv[1] == "-h":
        print "makemovie.py usage:"
        print "\tmakemovie.py <list of files to convert to a movie>"
        print "Wild cards make be used in the list."
        print "The output movie will be named \"test.mov\"."
        print "Example:"
        print "\t./makemovie.py *.ppm"
        print " "
        sys.exit()
        
frame = 1
for img in sys.argv[1:]:
    print str(frame) + " " + img 
    padframe = str(frame)
    if frame < 1000:
        padframe = "0" + padframe
    if frame < 100:
        padframe = "0" + padframe
    if frame < 10:
        padframe = "0" + padframe
    outimage = "conversion." + padframe + ".jpg"
    cmd = "convert " + img + " " + outimage
    os.system(cmd)
    frame = frame + 1

if frame > 1:
    cmd = "ffmpeg -r 24 -i conversion.%04d.jpg  -vcodec mjpeg -r 24 MyMovie.mov"
    os.system(cmd)
    cmd = "rm -rf conversion.????.jpg"
    os.system(cmd)
    cmd = "ffplay -loop 0 MyMovie.mov"
    os.system(cmd)
else:
    print "No frames found. No movie generated."

