#importing standard libraries
import os
import rospy
import rospkg
import sys

#importing qt libraries
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLineEdit, QMessageBox

#importing rospy msgs ---------------->
from std_msgs.msg import String

#--------------------------------------

class MyPlugin(Plugin):

    def __init__(self, context):
        
        #Creating a QObject
        super(MyPlugin, self).__init__(context)
        self.setObjectName('MyPlugin')   

        #Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true",
                        dest="quiet")
                        #help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ',unknowns

        #Creating QWidget
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('user_interface'), 'resource', 'user_interface_gui.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('MyPluginUi')

        #ROS publishers and subscribers--------------->
        self.coord_to_gui_sub = rospy.Subscriber('/coord_to_gui', String, self.comm_to_gui_callback)
        self.gui_to_coord_pub = rospy.Publisher('/gui_to_coord', String, queue_size=1000)

        #---------------------------------------------

        #GUI Button Connections---------------------------->
        self._widget.initialize_robot_btn.clicked[bool].connect(self.intialize_robots)

        ## Robot1--------------->
        self._widget.approach_wrkpiece_btn.clicked[bool].connect(self.approach_wrkpiece)
        self._widget.pick_workpiece_rob1_btn.clicked[bool].connect(self.pick_workpiece_rob1)
        self._widget.place_workpiece_btn.clicked[bool].connect(self.place_workpiece)

        ##Robot2---------------->
        self._widget.pick_workpiece_rob2_btn.clicked[bool].connect(self.pick_workpiece_rob2)
        self._widget.segregate_workpiece_btn.clicked[bool].connect(self.segregate_workpiece)

        #------------------------------------------------

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        
        context.add_widget(self._widget)

    ##Utility funcitons for User Interface
    def publish_msg_to_coord(self, msg):
        msg_to_publish  = msg
        self.gui_to_coord_pub.publish(msg_to_publish)

    def intialize_robots(self):
        self.publish_msg_to_coord('initialize_robots')
        pass

    ###Robot 1-----> 
    def approach_wrkpiece(self):
        self.publish_msg_to_coord('approach_workpiece_rob1')
        pass

    def pick_workpiece_rob1(self):
        self.publish_msg_to_coord('pick_workpiece_rob1')
        pass

    def place_workpiece(self):
        self.publish_msg_to_coord('place_workpiece_rob1')
        pass


    ###Robot 2------>
    def pick_workpiece_rob2(self):
        self.publish_msg_to_coord('pick_workpiece_rob2')
        pass

    def segregate_workpiece(self):
        self.publish_msg_to_coord('segregate_workpiece_rob2')
        pass

    ###Update gui status------->
    def comm_to_gui_callback(self,msg):
        status = msg.data
        self._widget.gui_status_txtbx.setText(status)
        pass


    ###Rqt functions
    def shutdown_plugin(self):
        #Unregister all publishers and subscribers
        self.gui_to_coord_pub.unregister()
        self.coord_to_gui_sub.unregister()
        pass
    
    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass
    
    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass