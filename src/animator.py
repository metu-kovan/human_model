#!/usr/bin/env python
import roslib
roslib.load_manifest('human_model')
import rospy
import json
import tf
import numpy
from abc import ABCMeta,abstractmethod
from tf.transformations import quaternion_multiply as quatMult,quaternion_conjugate
from collections import deque,defaultdict,OrderedDict

"""Module for converting tf data to construct a human model"""

def Vec(*args):
	"""returns a vector (numpy float array) with the length of number of given arguments"""
	return numpy.array(args,dtype=float)


def normalize(v):
	"""returns unit vector or quaternion"""
	return v/numpy.linalg.norm(v)



def quatRotatePoint(q,p,o=Vec(0,0,0)):
	"""returns point p rotated around quaternion q with the origin o (default (0,0,0)"""
	return quatMult(
		quatMult(q,numpy.append(p-o,(0,))),
		quaternion_conjugate(q)
	)[:3]+o
	
def calculateQuaternion(v1,v2):
	"""calculates the quaternion for rotating v1 to v2. Note that both v1 and v2 must be unit vector"""
	cross=numpy.cross(v1,v2)
	return normalize(numpy.append(cross,(1+numpy.dot(v1,v2),)))
	

class AveragePosition(object):
	"""Example Position Class
	
Calculates the average of the last n positions lazily

Calculated value can be accessed or changed via pos attribute:
p=AveragePosition(10)
p.pos+=Vec(1,2,3)
print(p.pos)

If an alternative position class is needed to be defined these functions,
must be defined in class:

@property
def pos(self):
	...

@pos.setter
def pos(self,p):
	...
	
def append(self,p):
	...
"""

	def __init__(self,n=100):
		self.transformations=deque((Vec(0,0,0),),n)
		self.calculated=None
		
	@property
	def pos(self):
		if self.calculated is None:
			self.calculated=numpy.average(self.transformations,0)
		return self.calculated
		
	@pos.setter
	def pos(self,p):
		self.calculated=p

	"""appends the given position p to the deque, and resets the calculated
average value"""
	def append(self,p):
		self.calculated=None
		self.transformations.append(p)
		
class JointTree(object):
	"""Recursive data structure to define joint tree.It have following attributes:
length:distance to the parent (if not fixed frame)
fixedFrame:fixates the point to the fixedFrame+the displacement of the tree
invert:inverts the rotating axis for connected limb
displacement:used to preserve the position of the node with respect to its parent(resets on new position)
limbPos:position of the limb(resets on new position)
limbRot:orientation of the limb(resets on new position)
	"""
	

	def toDict(self,ordered=False):	
		"""Converts tree to dictionary which can be exported as JSON,if ordered is true
it returns an OrderedDict instead of dictionary and preserves the order of attributes"""
		d=OrderedDict if ordered else dict
		return d(
				((
					self.name,
					d((
						('length',self.length),
						('invert',self.invert),
						('fixedFrame',None if self.fixedFrame is None else tuple(self.fixedFrame)),
						('children',tuple(i.toDict(ordered) for i in self.children)),
					))
				),))
				

	@staticmethod
	def fromDict(dictionary,pos):	
		"""converts a dictionary to JointTree"""
		(k,v)=next(iter(dictionary.items()))
		return JointTree(k,pos,**v)
		


	def __init__(self,name,posFunc,**kwargs):
		"""gets the name of the node and a function takes no argument,returns a Position class 	
(e.g. lambda : AveragePosition(10). It takes these optional arguments with the following default values:
length=0
invert=False
fixedFrame=None
children=[] (it can contain either dictionary or JointTree)
	"""
		self.name=name
		self.currentPos=posFunc()
		self.length=kwargs.get("length",0)
		self.invert=kwargs.get("invert",False)
		fixedFrame=kwargs.get("fixedFrame",None)
		self.fixedFrame=None if fixedFrame is None else Vec(*fixedFrame) 
		self.children=[]
		children=kwargs.get("children",[])
		try:
			if isinstance(children[0],dict):
				for i in children:
					(k,v)=next(iter(i.items()))
					self.addChild(JointTree(k,posFunc,**v))
			else:
				for i in children:
					self.addChild(i)
		except IndexError:
			pass
		self.parent=None
		self.__uncalculate()
		
	def __uncalculate(self):
		self.displacement=Vec(0,0,0)
		self.limbPos=Vec(0,0,0)
		self.limbRot=Vec(0,0,0,1)
		
	def __iter__(self):
		"""iterates over tree depth-first order"""
		yield self
		for i in self.children:
			for j in iter(i):
				yield j
				
	def __getitem__(self,name):
		"""returns the node with the given name, it raises a KeyError if there is no match"""
		for c in self:
			if c.name==name:
				return c
		raise KeyError("There is no node in tree with '{}' name".format(name))

	def addChild(self,child):
		"""adds new node to the tree"""
		child.parent=self
		self.children.append(child)

	def collectPosition(self,ls):
		"""gets the position of the joints from tf.TransformListener ls. It does nothing if there is no sent pose"""
		try:
			(trans,_)=ls.lookupTransform('/world',self.name,rospy.Time(0))
		except tf.Exception as e:
			return
		self.currentPos.append(Vec(*trans))
		self.__uncalculate()

	def setPosition(self):
		"""calculates the position of the joint"""
		if self.fixedFrame is not None:
			self.displacement+=self.fixedFrame-self.currentPos.pos
			self.currentPos.pos+=self.displacement
		elif self.parent is not None:
			n=self.currentPos.pos+self.displacement
			p=self.parent.currentPos.pos
			n=normalize(n-p)*self.length+p
			self.displacement=n-self.currentPos.pos
			self.currentPos.pos=n		
		for i in self.children:
			i.displacement+=self.displacement
		self.displacement=Vec(0,0,0)

	def connectLimbs(self):
		"""calculates the pose of the limbs"""
		p=self.currentPos.pos
		for i in self.children:
			c=i.currentPos.pos
			i.limbPos=(p+c)/2
			v2=normalize((p-c) if not i.invert else (c-p))
			i.limbRot=calculateQuaternion(Vec(0,0,1),v2)

	def sendPoses(self,br):
		"""sends the pose of joints and limbs to given tf.TransformBroadcaster"""
		br.sendTransform(self.currentPos.pos,(0,0,0,1),rospy.Time.now(),self.name+'_link','/world')
		for i in self.children:
			br.sendTransform(i.limbPos,i.limbRot,rospy.Time.now(),"{}_{}".format(self.name,i.name),'/world')
	
	def applyDisplacement(self,displacement):
		"""applies the given displacement to the parent and all of its children"""
		for i in self:
			i.currentPos.pos+=displacement
			i.limbPos+=displacement

if __name__ == '__main__':
	rospy.init_node('animator')
	treeDict=json.loads(rospy.get_param("/tree"))
	tree=JointTree.fromDict(treeDict,lambda : AveragePosition(10))
	br = tf.TransformBroadcaster()
	ls = tf.TransformListener()
	rate = rospy.Rate(50.0)
	while not rospy.is_shutdown():
		for i in tree:
			i.collectPosition(ls)

		for i in tree:
			i.setPosition()
			(o,r,l) = ("SpineShoulder","ShoulderRight","ShoulderLeft")
			#these three are special condition,They are aligned on a straight line
			#Also note that the z value of ShoulderRight and ShoulderLeft equals to that of SpineShoulder
			if i.name==o:
				r=i[r]
				l=i[l]
				cr=r.currentPos.pos+r.displacement
				cl=l.currentPos.pos+l.displacement
				cr[2]=i.currentPos.pos[2]
				cl[2]=i.currentPos.pos[2]
				k=i.currentPos.pos-(cr+cl)/2
				cr+=k
				cl+=k
				r.displacement=cr-r.currentPos.pos
				l.displacement=cl-l.currentPos.pos
				
		for i in tree:
			i.connectLimbs()
		
		
		#calculates the Orientation of Torso (Upper and Lower) and connected joints
		
		q1=tree["SpineShoulder"].limbRot
		q2=calculateQuaternion(Vec(0,1,0),normalize(tree["ShoulderRight"].currentPos.pos-tree["ShoulderLeft"].currentPos.pos))
		tree["SpineShoulder"].limbRot=quatMult(q2,q1)
		tree["ShoulderRight"].applyDisplacement(quatRotatePoint(q1,tree["ShoulderRight"].currentPos.pos,tree["SpineShoulder"].currentPos.pos)-tree["ShoulderRight"].currentPos.pos)
		tree["ShoulderLeft"].applyDisplacement(quatRotatePoint(q1,tree["ShoulderLeft"].currentPos.pos,tree["SpineShoulder"].currentPos.pos)-tree["ShoulderLeft"].currentPos.pos)		
		v=tree["HipRight"].currentPos.pos-tree["HipLeft"].currentPos.pos
		
		
		q2=calculateQuaternion(Vec(0,1,0),normalize(v))
		q=quatMult(q2,q1)
		tree["SpineBase"].limbRot=q
		tree["HipRight"].applyDisplacement(quatRotatePoint(q,tree["SpineBase"].currentPos.pos+Vec(0.01,tree["HipRight"].length,-0.05),tree["SpineBase"].currentPos.pos)-tree["HipRight"].currentPos.pos)
		tree["HipLeft"].applyDisplacement(quatRotatePoint(q,tree["SpineBase"].currentPos.pos+Vec(0.01,-tree["HipLeft"].length,-0.05),tree["SpineBase"].currentPos.pos)-tree["HipLeft"].currentPos.pos)	

		for i in tree:
			i.sendPoses(br)
		rate.sleep()
