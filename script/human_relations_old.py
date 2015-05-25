#!/usr/bin/env python

import sys, time
import rospy
import roslib
#from rospy import logerr, loginfo, Time
import json
import mysql.connector
from humans_msgs.msg import Humans
from geometry_msgs.msg import PointStamped

import tf

import datetime
import locale


config = {
    'user': 'root',
    'password': 'robot15',
    'host': '127.0.0.1',
    'db': 'data1',
    'charset':'utf8'
}

class human_relations:

    def __init__(self):
        self.t_name = ''
        self.con = mysql.connector.connect(**config)
        self.cur = self.con.cursor()
        #self.rel_sub = rospy.Subscriber("/humans/recog_info", Humans, self.relations_sub)
        self.rel_srv = rospy.Service('relation', Int32Srv, self.request)
        #self.tfl = tf.TransformListener()

        self.db_init()

    def datetime(self):
        d = datetime.datetime.today()
        return str(d.year)+str(d.month)+str(d.day)

    def db_init(self):
        date = self.datetime()
        qy = "create table `"+date+"-relations` (id int)"
        
        try:
            print qy
            self.cur.execute(qy)
        except mysql.connector.Error as err:
            if err.errno == mysql.connector.errorcode.ER_TABLE_EXISTS_ERROR:
                print "already exists."
            else:
                print err 

    def transform_pos(self, pt, hdr):
        ps = PointStamped()
        ps.point = pt
        ps.header = hdr
        #ps.header.stamp = hdr.stamp
        print "set:"+str(ps)
        self.tfl.waitForTransform(ps.header.frame_id, '/map', ps.header.stamp, rospy.Duration(3.0))
        point_in_map = self.tfl.transformPoint('/map', ps)
        return point_in_map

    def relations_sub(self,hums):
        #print hums
        self.tfl.waitForTransform("/map", "/camera_link", rospy.Time(), rospy.Duration(4.0))
        for hum in hums.human:
            print hum.p
            tl_ps = PointStamped()
            tl_ps = self.transform_pos(hum.p, hum.header)
            #print tl_p
        #for hum in hums:
         #   print hums

def main(args):
    hr = human_relations()
    rospy.init_node('human_relations', anonymous=True)
    try:
        rospy.spin()
    except KeybordInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
