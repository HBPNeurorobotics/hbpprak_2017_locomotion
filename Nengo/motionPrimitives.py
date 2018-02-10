import nengo
import rospy
import numpy as np	
from std_msgs.msg import Float64

class MotionPrimitives():
	def __init__(self, primitiveType, stim, joints, pubOnlyJoints = []):
		self._primitiveType = primitiveType
		self._stim = stim
		self._outValues = []
		self._joints = joints
		self._pubOnlyJoints = pubOnlyJoints
		
		#the values of joints will be published to the pubjoints aswell (pairwise)
		self._all_joints = []
		for i in range(len(self._joints)):
			for j in range(len(self._joints[i])):
				self._all_joints.append(self._joints[i][j])
		for i in range(len(self._pubOnlyJoints)):
			for j in range(len(self._pubOnlyJoints[i])):
				self._all_joints.append(self._pubOnlyJoints[i][j])
				
		self._unique_joints = sorted(set(self._all_joints), key= self._all_joints.index)
		
	
		self._joints_pub = [[] for y in range (len(self._all_joints)+1)]
		for i in self._unique_joints:
			self._joints_pub[0].append(i)
			self._joints_pub[1].append(rospy.Publisher(i, Float64, queue_size=1))
				
		
		#Indices and Values (40 samples)		
		self._indices_40 = np.array([0. , 0.03, 0.05,  0.08,  0.1,   0.13,  0.15,  0.18,  0.21,  0.23,  0.26,  0.28, 0.31,  0.33,  0.36,  0.38,  0.41,  0.44,  0.46,  0.49,  0.51,  0.54,  0.56,  0.59, 0.62,  0.64,  0.67,  0.69,  0.72,  0.74,  0.77,  0.79,  0.82, 0.85, 0.87, 0.9,  0.92, 0.95, 0.97, 1.])
		self._indices_80 = np.array([0.00, 0.01, 0.03, 0.04, 0.05, 0.06, 0.08, 0.09, 0.10, 0.11, 0.13, 0.14, 0.15, 0.16, 0.18, 0.19, 0.20, 0.22, 0.23, 0.24, 0.25, 0.27, 0.28, 0.29, 0.30, 0.32, 0.33, 0.34, 0.35, 0.37, 0.38, 0.39, 0.41, 0.42, 0.43, 0.44, 0.46, 0.47, 0.48, 0.49, 0.51, 0.52, 0.53, 0.54, 0.56, 0.57, 0.58, 0.59, 0.61, 0.62, 0.63, 0.65, 0.66, 0.67, 0.68, 0.70, 0.71, 0.72, 0.73, 0.75, 0.76, 0.77, 0.78, 0.80, 0.81, 0.82, 0.84, 0.85, 0.86, 0.87, 0.89, 0.90, 0.91, 0.92, 0.94, 0.95, 0.96, 0.97, 0.99, 1.00])
		self._values_0_1 =np.array([  0. , 0. ,  0.01,  0.01,  0.03,  0.04,  0.06,  0.08,  0.1,   0.13, 0.15, 0.18, 0.22, 0.25, 0.29, 0.32, 0.36, 0.4,0.44, 0.48, 0.52, 0.56, 0.6, 0.64, 0.68, 0.71, 0.75, 0.78, 0.82, 0.85, 0.87, 0.9, 0.92, 0.94, 0.96, 0.97, 0.99, 0.99, 1.  , 1.])
		self._values_1_0 = np.array([1.0, 1.0, 0.99, 0.99, 0.97, 0.96, 0.94, 0.92, 0.9, 0.87, 0.85, 0.82, 0.78, 0.75, 0.71, 0.68, 0.64, 0.6, 0.56, 0.52, 0.48, 0.44, 0.4, 0.36, 0.32, 0.29, 0.25, 0.22, 0.18, 0.15, 0.13, 0.1, 0.08, 0.06, 0.04, 0.03, 0.01, 0.01, 0.0, 0.0])
		
		self._values_0_1_0 = np.array([0.0, 0.0, 0.01, 0.01, 0.03, 0.04, 0.06, 0.08, 0.1, 0.13, 0.15, 0.18, 0.22, 0.25, 0.29, 0.32, 0.36, 0.4, 0.44, 0.48, 0.52, 0.56, 0.6, 0.64, 0.68, 0.71, 0.75, 0.78, 0.82, 0.85, 0.87, 0.9, 0.92, 0.94, 0.96, 0.97, 0.99, 0.99, 1.0, 1.0, 1.0, 1.0, 0.99, 0.99, 0.97, 0.96, 0.94, 0.92, 0.9, 0.87, 0.85, 0.82, 0.78, 0.75, 0.71, 0.68, 0.64, 0.6, 0.56, 0.52, 0.48, 0.44, 0.4, 0.36, 0.32, 0.29, 0.25, 0.22, 0.18, 0.15, 0.13, 0.1, 0.08, 0.06, 0.04, 0.03, 0.01, 0.01, 0.0, 0.0])
		
		self._values_1_0_1 = np.array([1.0, 1.0, 0.99, 0.99, 0.97, 0.96, 0.94, 0.92, 0.9, 0.87, 0.85, 0.82, 0.78, 0.75, 0.71, 0.68, 0.64, 0.6, 0.56, 0.52, 0.48, 0.44, 0.4, 0.36, 0.32, 0.29, 0.25, 0.22, 0.18, 0.15, 0.13, 0.1, 0.08, 0.06, 0.04, 0.03, 0.01, 0.01, 0.0, 0.0, 0.0, 0.0, 0.01, 0.01, 0.03, 0.04, 0.06, 0.08, 0.1, 0.13, 0.15, 0.18, 0.22, 0.25, 0.29, 0.32, 0.36, 0.4, 0.44, 0.48, 0.52, 0.56, 0.6, 0.64, 0.68, 0.71, 0.75, 0.78, 0.82, 0.85, 0.87, 0.9, 0.92, 0.94, 0.96, 0.97, 0.99, 0.99, 1.0, 1.0])

		self._data_0_1 = np.array([self._indices_40, self._values_0_1])
		self._data_1_0 = np.array([self._indices_40, self._values_1_0])
		self._data_0_1_0 = np.array([self._indices_80, self._values_0_1_0])
		self._data_1_0_1 = np.array([self._indices_80, self._values_1_0_1])

		if self._primitiveType == 0:
			self._trajectory = np.array([self._data_0_1, self._data_1_0])
			self._num_joints = 2
			self._min = [-0.25,-0.25]
			self._max = [0.25,0.25]
		elif self._primitiveType == 1:
			self._trajectory = np.array([self._data_0_1_0, self._data_1_0_1])
			self._num_joints = 2
			self._min = [1.5, -3.14]
			self._max = [3.14, -2.0]
		elif self._primitiveType == 2:
			self._trajectory = np.array([self._data_1_0])
			self._num_joints = 1
			self._min = [1.0]
			self._max = [1.5]
		else:
			self._trajectory = []
			self._num_joints = 0
			self._min = 0
			self._max = 0
			#ERROR

	#map stimulus [0;1] to trajectory
	def map_trajectory(self, x):
		res = []
		for i in range(len(self._trajectory)):
			dataset = self._trajectory[i]
			u_indice = dataset[0 , (np.abs(dataset[0] - x)).argmin()]
			u_index = np.where(dataset[0] == u_indice)
			res.append(dataset[1, u_index])
		return res

	#map to min and max value and scale
	def map(self, u):
		res = []
		for i in range(self._num_joints):
			tmp = u[i]
			tmp = tmp * (self._max[i] - self._min[i])
			tmp = tmp + self._min[i]
			res.append(tmp)
		return res
	
	#publish topic 
	def publish_topic(self, t, x):
		z = self._num_joints
		for i in range(len(self._joints_pub[1])):
			self._joints_pub[1][i].publish(x[i%z])
		return True
							
	# returns the network
	def get_network(self):
		net = nengo.Network()
		with net:
			#Nodes
			net.stim = nengo.Node(self._stim, label='Stimulus')
			net.input = nengo.Node(size_in=1)
			net.output = nengo.Node(size_in=self._num_joints)
			net.ros_out = nengo.Node(self.publish_topic, size_in=self._num_joints)

			#Ensembles
			net.u = nengo.Ensemble(n_neurons=200, dimensions=1, label='u')
			net.fu = nengo.Ensemble(n_neurons=200, dimensions=self._num_joints, radius=2, label='f(u)')
			net.gfu = nengo.Ensemble(n_neurons=100, dimensions=self._num_joints, radius=2,  neuron_type=nengo.Direct(), label='g(f(u))')		
			#Connections
			nengo.Connection(net.stim[0], net.input)		
			nengo.Connection(net.input, net.u)		
			nengo.Connection(net.u, net.fu, function=self.map_trajectory)
			nengo.Connection(net.fu, net.output, function=self.map)

			for i in range(self._num_joints):
				nengo.Connection(net.output[i], net.gfu[i])
			nengo.Connection(net.gfu, net.ros_out)
						
		return net

	def change_stim(self, newStim):
		self._stim = newStim


									
									
									
			