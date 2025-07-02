#! /usr/bin/env python
import rospkg
import rospy
from opencv_apps.msg import FaceArrayStamped
from std_msgs.msg import String
from datetime import datetime
import os
import csv
from gtts import gTTS

class RobotInteractionManager:
    def __init__(self):
        self.food = {"sushi": 3, "fries": 2,"water":1}
        self.bill = 0
        self.person_name = None
        self.confidence_level = None

        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path('robot_interaction')
        except rospkg.ResourceNotFound:
            rospy.logerr("Package 'robot_interaction' not found. Make sure it's built and sourced.")
            return

        counter_file_name = "bill_id_counter.txt"
        self.counter_file_path = os.path.join(package_path, 'data', counter_file_name)

        db_file_name = "bill_database.csv"
        self.db_file_path = os.path.join(package_path, 'data', db_file_name)
        
        rospy.init_node("interaction_manager_node",anonymous=True)
        rospy.loginfo("Food ordering manager started")

        rospy.Subscriber(
            "/face_recognition/output", 
            FaceArrayStamped, 
            self.face_detection_callback
            )
        rospy.Subscriber(
            "/google_sr", 
            String, 
            self.speech_callback
            )


    # def speak(self, data):
    #     tts = gTTS(data)
    #     tts.save("speech.mp3")
    #     os.system("mpg321 speech.mp3")
    #     os.remove("speech.mp3")

    def speech_callback(self, msg):
        transcribed_data = msg.data

        food_ordered = transcribed_data.lower().strip()
        
        if food_ordered in self.food:
            self.bill += self.food[food_ordered]
            rospy.loginfo(f"{food_ordered} ordered with price {self.food[food_ordered]}")
            
        elif food_ordered == "done":
            rospy.loginfo(f"Your bill will be RM{self.bill}")
            # self.speak(f"Your bill will be RM{self.bill}")
            self.save_bill()
            self.bill = 0
        else:
            rospy.loginfo(f"{food_ordered} is not in our menu")

    def face_detection_callback(self, msg):
        if not msg.faces:
            rospy.loginfo("No identified user showed in the frame")
            return
        
        for face in msg.faces:
            if face.label:
                self.person_name = face.label
                self.confidence_level = face.confidence

            rospy.loginfo(f"we can see {self.person_name} with confidence level {self.confidence_level}")

    def save_bill(self):
        try:
            with open(self.counter_file_path,'r') as f:
                content=f.read()
                if content:
                    current_id = int(content)
        except(FileNotFoundError):
            rospy.logerr("File not found or corrupted, starting from 0")
            current_id = 0
        
        new_id = current_id + 1

        with open(self.counter_file_path,"w") as f:
            f.write(str(new_id))

        field_names= ["Bill_ID","Customer_Name","Total_Price","Time_Created"]
        time_created= datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        new_row = {
            "Bill_ID": current_id,
            "Customer_Name" : self.person_name,
            "Total_Price": self.bill,
            "Time_Created" : time_created
        }

        file_exists = os.path.exists(self.db_file_path)

        with open(self.db_file_path,"a", newline='') as csvfile:
            writer = csv.DictWriter(csvfile,fieldnames=field_names)

            if not file_exists:
                writer.writeheader()
            writer.writerow(new_row)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    interaction_manager = RobotInteractionManager()
    interaction_manager.run()
