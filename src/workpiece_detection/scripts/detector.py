#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import cv2
import time
import sys
import numpy as np

INPUT_WIDTH = 640
INPUT_HEIGHT = 640
SCORE_THRESHOLD = 0.2
NMS_THRESHOLD = 0.4
CONFIDENCE_THRESHOLD = 0.4
WIDTH_THRESHOLD = 120


colors = [(255, 255, 0), (0, 255, 0), (0, 255, 255), (255, 0, 0)]

is_cuda = len(sys.argv) > 1 and sys.argv[1] == "cuda"

#net = build_model(is_cuda)
#capture = load_capture()

#start = time.time_ns()
frame_count = 0
total_frames = 0
fps = -1


class WorkpieceDetector :
    def __init__(self):
        self.frame_count = 0
        self.total_frames = 0
        self.fps = -1
        #self.start = time.time_ns()
        self.boxes = None

        self.pub = rospy.Publisher('/detection', Twist, queue_size=10)
        self.velocity_msg = Twist()

    def build_model(self , is_cuda):
        self.net = cv2.dnn.readNet("best_epoch_50.onnx")
        if is_cuda:
            print("Attempty to use CUDA")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        else:
            print("Running on CPU")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        return self.net



    def detect(self,image,net ):
        self.net = net
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
        self.net.setInput(blob)
        preds = self.net.forward()
        return preds

    def load_capture(self):
        self.capture = cv2.VideoCapture(2)
        return self.capture

    def load_classes(self):
        class_list = []
        with open("classes.txt", "r") as f:
            self.class_list = [cname.strip() for cname in f.readlines()]
        return self.class_list



    def wrap_detection(self,input_image, output_data):
        class_ids = []
        confidences = []
        self.boxes = []

        rows = output_data.shape[0]

        image_width, image_height, _ = input_image.shape

        x_factor = image_width / INPUT_WIDTH
        y_factor =  image_height / INPUT_HEIGHT

        

        for r in range(rows):
            row = output_data[r]
            confidence = row[4]
            if confidence >= 0.4:

                classes_scores = row[5:]
                _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
                class_id = max_indx[1]
                if (classes_scores[class_id] > .25):

                    confidences.append(confidence)

                    class_ids.append(class_id)

                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                    left = int((x - 0.5 * w) * x_factor)
                    top = int((y - 0.5 * h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    box = np.array([left, top, width, height])
                    self.boxes.append(box)

        indexes = cv2.dnn.NMSBoxes(self.boxes, confidences, 0.25, 0.45) 

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(self.boxes[i])

        return result_class_ids, result_confidences, result_boxes

    def format_yolov5(self,frame):

        row, col, _ = frame.shape
        _max = max(col, row)
        result = np.zeros((_max, _max, 3), np.uint8)
        result[0:row, 0:col] = frame
        return result

    def move(self,result) :
        self.velocity_msg.linear.x = result
        self.pub.publish(self.velocity_msg)
    
    def control_loop(self , frame):

        self.xcenter = frame.shape[0]/2
        self.ycenter = frame.shape[1]/2

        #cv2.putText(frame , "( {} , {} )".format(int(self.xcenter) , int(self.ycenter)) , (int(self.xcenter) , int(self.ycenter)) , cv2.FONT_HERSHEY_SIMPLEX , 1,(255,0,0) , 2 ,cv2.LINE_AA)

        cv2.line(frame , (int(self.xcenter),0) , (int(self.xcenter),int(self.ycenter)*2) , (255,0,0) , 5)
        angular_text = None
        orient_text = None

        try :

            obj_width = self.boxes[0][2]
            obj_height = self.boxes[0][3]

            obj_xcenter = self.boxes[0][0]  + obj_width/2
            obj_ycenter = self.boxes[0][1] + obj_height/2

            #cv2.putText(frame , "( {} , {} )".format(int(obj_xcenter) , int(obj_ycenter)) , (int(obj_xcenter) , int(obj_ycenter)) , cv2.FONT_HERSHEY_SIMPLEX , 1,(255,0,0) , 2 ,cv2.LINE_AA)
            
            cv2.circle(frame, (int(obj_xcenter) , int(obj_ycenter)), 5, (255,0,0), 5)

            if self.xcenter > obj_xcenter + obj_width/2:
                angular_text = "<<<===== LEFT"
                self.move(-1)
            elif self.xcenter < obj_xcenter - obj_width/2:
                angular_text = "=====>>> RIGHT"
                self.move(1)
            else:
                angular_text = "CENTER"
                self.move(0)

            if obj_width > WIDTH_THRESHOLD + 15 :
                orient_text = "ROTATE"
            else:
                orient_text = "ALLIGNED"
        except :
            #pass
            print("except")


        cv2.putText(frame , angular_text , (200 , 50 ) , cv2.FONT_HERSHEY_SIMPLEX , 1,(255,0,0) , 2 ,cv2.LINE_AA)
        cv2.putText(frame , orient_text , (200 , 400 ) , cv2.FONT_HERSHEY_SIMPLEX , 1,(255,0,0) , 2 ,cv2.LINE_AA)





    def run(self) :
        self.net = self.build_model(is_cuda)
        self.capture = self.load_capture()

        while True:
        
            _, frame = self.capture.read()
            if frame is None:
                print("End of stream")
                break
            
            inputImage = self.format_yolov5(frame)
            #resized = cv2.resize(inputImage , (640,640))
            #blurred = cv2.blur(resized ,(10,10))
            outs = self.detect(inputImage, self.net)

            class_ids, confidences, self.boxes = self.wrap_detection(inputImage, outs[0])
            #print("ID : " , class_ids)
            print("Boxes : ",self.boxes)

            self.frame_count += 1
            self.total_frames += 1

            for (classid, confidence, box) in zip(class_ids, confidences, self.boxes):
                 color = colors[int(classid) % len(colors)]
                 cv2.rectangle(inputImage, box, color, 2)
                 cv2.rectangle(inputImage, (box[0], box[1] - 20), (box[0] + box[2], box[1]), color, -1)

                 try :
                     cv2.putText(frame, self.class_list[classid], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))
                 except :
                     pass
            self.control_loop(inputImage)
                 
            if self.frame_count >= 30:
                #self.end = time.time_ns()
                #self.fps = 1000000000 * frame_count / (self.end - self.start)
                self.frame_count = 0
                #self.start = time.time_ns()

            '''if self.fps > 0:
                self.fps_label = "FPS: %.2f" % self.fps
                cv2.putText(frame, fps_label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            '''
            cv2.imshow("output", inputImage)

            if cv2.waitKey(1) > -1:
                print("finished by user")
                break

#print("Total frames: " + str(total_frames))

def main():
    rospy.init_node('workpiece_detection' , anonymous=True)
    try :

        wd = WorkpieceDetector()
        wd.run()
    except:

        cv2.destroyAllWindows()


main()
cv2.destroyAllWindows()
