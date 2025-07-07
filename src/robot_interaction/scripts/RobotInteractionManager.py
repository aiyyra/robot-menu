#! /usr/bin/env python
import random
import rospkg
import rospy
from opencv_apps.msg import FaceArrayStamped
from std_msgs.msg import String
from datetime import datetime
import os
import json
from gtts import gTTS

class RobotInteractionManager:
    def __init__(self):
        self.food = {"sushi": 3, "fries": 2,"water":1, "burger": 4, "coffee": 1}
        self.bill = 0
        self.person_name = None
        self.confidence_level = None

        self.bill_counter_path = self.getPath("robot_interaction", "bill_id_counter.txt")
        self.db_bill_path = self.getPath("robot_interaction", "bill_database.json")

        self.unique_customer = {}
        self.bill_session = []

        self.create_bill_session()


        
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

    def getPath(self, package, file):
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path(package)
        except rospkg.ResourceNotFound:
            rospy.loginfo(f"Package '{package}' not found'")
        path = os.path.join(package_path, "data", file)
        return path

    def speak(self, data):
        tts = gTTS(data)
        tts.save("speech.mp3")
        os.system("mpg321 speech.mp3")
        os.remove("speech.mp3")

    def speech_callback(self, msg):
        text_data = msg.data

        food_ordered = text_data.lower().strip()
        
        if food_ordered in self.food and self.person_name:
            # update order
            price = self.food[food_ordered]
            self.update_single_person_order(self.person_name, food_ordered, price)
            rospy.loginfo(self.bill_session)
            self.speak(f"{food_ordered} added to order for {str(self.person_name).split('_')[0]}")
            self.bill += price
            rospy.loginfo(f"{food_ordered} ordered with price {price}")
            
        elif food_ordered == "done":
            bill = self.bill_session["table_order"]
            for person in bill:
                name = str(person["name"]).split("_")[0]
                order = []
                temp_bill = 0
                for item in person["order"]:
                    order.append(item['item'])
                    temp_bill += self.food[item['item']]
                rospy.loginfo(f"Name:{name}\nOrder:{order}")

                if order == []: continue
                self.speak(f"{name} order {order}, your bill will be RM{temp_bill}")
            
            self.speak(f"Your overall bill will be RM{self.bill}")
            self.save_bill()
            self.bill = 0
        else:
            temp_str = str(food_ordered).split(" ")
            for x in temp_str :
                if x == "done": 
                    bill = self.bill_session["table_order"]
                    for person in bill:
                        name = str(person["name"]).split("_")[0]
                        order = []
                        temp_bill = 0
                        for item in person["order"]:
                            order.append(item['item'])
                            temp_bill += self.food[item['item']]
                        rospy.loginfo(f"Name:{name}\nOrder:{order}")

                        if order == []: continue
                        self.speak(f"{name} order {order}, your bill will be RM{temp_bill}")
                    
                    self.speak(f"Your overall bill will be RM{self.bill}")
                    self.save_bill()
                    self.bill = 0


            rospy.loginfo(f"{food_ordered} is not in our menu")

    def is_unique(self, customer):
        return not customer in self.unique_customer.keys()
    
    def face_detection_callback(self, msg):
        if not msg.faces:
            rospy.loginfo("No identified user showed in the frame")
            return
        
        for face in msg.faces:
            if face.label:
                if self.is_unique(face.label):
                    self.unique_customer[face.label]=0
                    self.add_person_to_table(face.label)
                    self.speak(f"Hello {str(face.label).split('_')[0]},")
                    
                    recommendation = self.getLatestOrder(face.label)
                    if recommendation is not None:
                        name = str(face.label).split("_")[0]
                        self.speak(f"would you like some {recommendation}")
                self.person_name = str(face.label)
                self.confidence_level = face.confidence
                

            # rospy.loginfo(f"we can see {self.person_name} with confidence level {self.confidence_level}")

    def update_single_person_order(self, person_name, item, price):
        # Find the person by name
        for person in self.bill_session["table_order"]:
            if person["name"] == person_name:
                new_order = {"item": item}
                person["order"].append(new_order)
                self.unique_customer[person_name] += price

    def add_person_to_table(self, name):
        new_person = {
            "name": name, 
            "time_created": datetime.now().strftime("%Y-%m-%d %H:%M:%S"), 
            "order":[]
            }
        rospy.loginfo(new_person)

        self.bill_session["table_order"].append(new_person)

        rospy.loginfo(self.bill_session)

    def create_bill_session(self):
        bill_id = self.getId()

        bill_session_new = {"bill_id": bill_id, "table_order":[]}
        self.bill_session =bill_session_new 

    def getId(self):
        rospy.loginfo(f"Attempting to update id")
        try:
            with open(self.bill_counter_path,'r') as f:
                content=f.read()
                if content:
                    current_id = int(content)
        except(FileNotFoundError):
            rospy.logerr("File not found or corrupted, starting from 0")
            current_id = 0
        return current_id

    def save_bill(self):
        current_id = self.bill_session["bill_id"]
        new_id = current_id + 1

        with open(self.bill_counter_path,"w") as f:
            f.write(str(new_id))
            f.close()

        with open(self.db_bill_path, 'r') as f:
            try:
                data=json.load(f)
                f.close()
            except json.JSONDecodeError:
                rospy.logerr(f"Error reading data")
                data =[]
    
        try:
            with open(self.db_bill_path, 'w') as f:
                data.append(self.bill_session)
                json.dump(data, f, indent=4)
                f.close()
            rospy.loginfo(f"Data successfully saved to {self.db_bill_path}")
        except Exception as e:
            rospy.logerr(f"Error saving data to JSON: {e}")

        self.create_bill_session()

    def getLatestOrder(self, name):
        try:
            with open(self.db_bill_path, 'r') as f:
                try:
                    data = json.load(f)
                except json.JSONDecodeError:
                    rospy.logerr(f"Error reading data from {self.db_bill_path}")
                    return None
        except FileNotFoundError:
            rospy.logwarn(f"No file found at {self.db_bill_path}")
            return None

        # Collect all orders for the given name
        customer_orders = []

        for bill in data:
            for person in bill.get("table_order", []):
                if person.get("name") == name and person.get("order"):
                    try:
                        time_obj = datetime.strptime(person["time_created"], "%Y-%m-%d %H:%M:%S")
                    except Exception:
                        time_obj = datetime.min

                    customer_orders.append({
                        "time_created": time_obj,
                        "order": person["order"]
                    })

        if not customer_orders:
            rospy.loginfo(f"No orders found for {name}")
            return None

        # Find the latest time
        latest_time = max(o["time_created"] for o in customer_orders)

        # Get all orders with that timestamp (to allow randomness if tied)
        latest_orders = [o["order"] for o in customer_orders if o["time_created"] == latest_time]

        chosen_order = random.choice(latest_orders)
        item_names = [o["item"] for o in chosen_order]
        rospy.loginfo(f"Latest order for {name}: {item_names}")
        return item_names

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    interaction_manager = RobotInteractionManager()
    interaction_manager.run()
