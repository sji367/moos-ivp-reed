#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Apr  6 14:58:23 2017

@author: sji367
"""
import time
import os
import urllib2
from shutil import copyfile

class ImageSaver(object):
    """ This object  grabs the image off of an IP camera and saves it to 
         disc at a rate of 1/delaysec Hz or as fast as it can grab them, which
         ever is slower. 
    """
    def __init__(self):
        self.delaysecs = 1
        self.axisip = '192.168.1.6'
        self.log_dir = '../../pics'
        
    def save_image(self):
        """ This function grabs the image off of the IP camera and saves it to 
             disc.
            
            Outputs:
                run_time - total time to grab image and save it to disc
        """
        start_time = time.time()
        
        # Grab Image
        url = "http://%s/axis-cgi/jpg/image.cgi" % self.axisip
        # Store image data
        try:
            I = urllib2.urlopen(url)
            data = I.read()
        except:
            print "Error getting image at %s" % url
            data= ""
            
        # Save image to disc
        filename = os.path.join(self.log_dir,'image_%d.jpg' % int(start_time))
        fh = open(filename,'w')
        fh.write(data)
        fh.close()
        copyfile(filename,os.path.join(self.log_dir,'latest.jpg'))
        return time.time()-start_time
    
    def run(self):
        """ Saves images at 1/delaysec Hz or as fast as it can grab them, which
             ever is slower. 
        """
        while(True):
            run_time = self.save_image()
            if run_time<self.delaysecs:
                time.sleep(self.delaysecs-run_time)
        
if __name__ == '__main__':
    saver = ImageSaver()
    saver.run()
    