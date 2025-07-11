#! /usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Yuki Furuta.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Kei Okada nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from __future__ import print_function

try:
    input = raw_input
except:
    pass

import rospy
import message_filters
from sensor_msgs.msg import Image
from opencv_apps.msg import FaceArrayStamped
from opencv_apps.srv import FaceRecognitionTrain, FaceRecognitionTrainRequest

from datetime import datetime
import csv 
import os
import rospkg


class FaceRecognitionTrainer(object):
    def __init__(self):
        self.queue_size = rospy.get_param("~queue_size", 100)
        
        self.img_sub = message_filters.Subscriber("image", Image)
        self.face_sub = message_filters.Subscriber("faces", FaceArrayStamped)
        
        self.req = FaceRecognitionTrainRequest()
        self.label = ""
        self.name = ""
        self.ok = False
        
        self.counter_file_path = self.getPath("opencv_apps", "user_id_counter.txt")
        self.db_file_path = self.getPath("robot_interaction", "user_database.csv") 

        self.sync = message_filters.TimeSynchronizer([self.img_sub, self.face_sub],
                                                     self.queue_size)
        self.sync.registerCallback(self.callback)

    def getPath(self, package, file):
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path(package)
        except rospkg.ResourceNotFound:
            rospy.loginfo(f"Package '{package}' not found'")
        path = os.path.join(package_path, "data", file)
        return path
    def getId(self):
        rospy.loginfo(f"Attempting to update id")
        try:
            with open(self.counter_file_path,'r') as f:
                content=f.read()
                if content:
                    current_id = int(content)
        except(FileNotFoundError):
            rospy.logerr("File not found or corrupted, starting from 0")
            current_id = 0

        return current_id
    def save_user(self, name, id):
        field_names= ["Name","User_ID","Time_Created"]
        new_row = {
            "Name": name,
            "User_ID" : id,
            "Time_Created" :datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }

        file_exists = os.path.exists(self.db_file_path)
        with open(self.db_file_path,"a", newline='') as csvfile:
            writer = csv.DictWriter(csvfile,fieldnames=field_names)

            if not file_exists:
                writer.writeheader()
            writer.writerow(new_row)
            csvfile.close()
                    
        new_id = id + 1
        with open(self.counter_file_path,"w") as f:
            f.write(str(new_id))
            f.close()

    def callback(self, img, faces):
        if len(faces.faces) <= 0:
            return
        if self.ok:
            faces.faces.sort(key=lambda f: f.face.width * f.face.height)
            self.req.images.append(img)
            self.req.rects.append(faces.faces[0].face)
            self.req.labels.append(self.label)
            self.ok = False
    def run(self):
        rospy.wait_for_service("train")
        train = rospy.ServiceProxy("train", FaceRecognitionTrain)
        
        name = input("Please input your name and press Enter: ")
        id = self.getId()
        self.label= name + "_" + str(id)
        
        if len(name) <= 0 or input("Your name is %s. Correct? [y/n]: " % name) not in ["", "y", "Y"]:
            name = input("Please input your name and press Enter: ")

        input("Please stand at the center of the camera and press Enter: ")
        while True:
            self.ok = True
            while self.ok:
                print("taking picture...")
                rospy.sleep(1)
            if input("One more picture? [y/n]: ") not in ["", "y", "Y"]:
                break
        print("sending to trainer.. .")

        res = train(self.req)
        if res.ok:
            self.save_user(name, id)
            print("OK. Trained successfully!")
        else:
            print("NG. Error: %s" % res.error)

if __name__ == '__main__':
    rospy.init_node("face_recognition_trainer")
    t = FaceRecognitionTrainer()
    t.run()
