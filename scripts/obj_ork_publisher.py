#!/usr/bin/env python
# Python Libs
import numpy as np
# Ros Libs
import rospy
import tf
# Ros Messages
from object_recognition_msgs.msg import RecognizedObjectArray, RecognizedObject 
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Pose, PoseStamped

class object_recognition:
		"""
		Class that listens the object recognition message and publish each detected object
		seperately
		"""
		
		def __init__(self):
			self.sub = rospy.Subscriber('/recognized_object_array', RecognizedObjectArray, self.read)
			#self.rate = rospy.Rate(20) # Hz
			
			self.switcher = {
				'd5bd927be17de95dc70a2c726b000277': "cylinder", # 
				'3fefefe16a07ff5b76b7c7d813000bdb': "cylinder", # polygon 
				'4e37b85ae1a07dcb5243ee571d00841f': "cylinder",
				}
		
		def give_object_id(self,id_num):
			self.switcher = {
				id_num : "cylinder",
				}
		
		"""
		Read Function Subscribe to the object recognized topic and handles 
		the objects data.
		"""
		def read(self,obj):
			print "=============================="
			print "The detected objects:"
			print("Num of Objects = {0}".format(obj.objects.__len__()))
						
			# list with detected objects
			#data = [RecognizedObject() for i in range(obj.objects.__len__())]	
			# copy list
			data = obj.objects
			self.pub = [ [0 for k in range(2)] for m in range(data.__len__())]
			# for each object publish the desired topics
			
			
			for i in range(data.__len__()):
				name = self.ids_to_names(data[i].type.key)
				print("Id:[{0}] Name:[{1}]".format(data[i].type.key,name))
				
				self.pub[i][0] = rospy.Publisher('/'+name+'/points',PointCloud2, queue_size=10)
				self.pub[i][1] = rospy.Publisher('/'+name+'/pose',PoseStamped, queue_size=10)
				
				# If you want to publish the pose on tf
				#br = tf.TransformBroadcaster()
				#p = data[i].pose.pose.pose
				#print p
				#br.sendTransform((p.position.x, p.position.y,p.position.z),
				#( p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
				#									rospy.Time.now(),name,"robot_0/xtion_depth_optical_frame")
				
				# Set the Header to pointcloud message
				points = data[i].point_clouds[0]
				points.header = data[i].header
				
				# Set PoseStamped msg
				pose = PoseStamped()
				pose.header = points.header
				pose.pose = data[i].pose.pose.pose
				
				# publish point clouds
				self.pub[i][0].publish(points)
				# publish pose
				self.pub[i][1].publish(pose)
				
			#sleep sec
			#self.rate.sleep()
		
		
		
		"""
		Function which matches the object ids to the object names.
		"""
		def ids_to_names(self,arg):
#			self.switcher = {
#				'b48d7f38a0608df17b928a351100a58c': "aruco_cube",
#				'4e37b85ae1a07dcb5243ee571d0074ce': "box_6_6_40",
#				'4e37b85ae1a07dcb5243ee571d00841f': "cylinder",
#				}
			return self.switcher.get(arg,"nothing") # return "nothing" if arg not found

if __name__ == '__main__':
	
	import sys
	
	try:
		rospy.init_node('object_recognition_publisher',anonymous = True)
		obj = object_recognition()
		if (len(sys.argv)>1):
		  obj.give_object_id(sys.argv[1])
		while not rospy.is_shutdown():
			rospy.spin()
	except rospy.ROSInterruptException:
		pass

