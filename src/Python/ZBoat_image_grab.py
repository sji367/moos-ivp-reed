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
    def __init__(self):
        self.delaysecs = 1
        self.axisip = '192.168.1.6'
        self.log_dir = '../../pics'
        
    def get_data(self):
        time.sleep(self.delaysecs)
        url = "http://%s/axis-cgi/jpg/image.cgi" % self.axisip

        try:
            I = urllib2.urlopen(url)
        except:
            print "Error getting image at %s" % url
            return ""

        return I.read()
    
    def save_image(self, data):
        """ Save the image to disc """
        
        filename = os.path.join(self.log_dir,'image_%d.jpg' % int(time.time()))
        fh = open(filename,'w')
        fh.write(data)
        fh.close()
        copyfile(filename,os.path.join(self.log_dir,'latest.jpg'))
    
    def run(self):
        """ Saves images at 1/delaysec Hz """
        while(True):
            self.save_image(self.get_data())
        
if __name__ == '__main__':
    saver = ImageSaver()
    saver.run()
    