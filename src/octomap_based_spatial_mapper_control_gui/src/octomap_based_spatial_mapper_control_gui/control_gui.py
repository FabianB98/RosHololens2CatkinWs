import os
import rospy
import rospkg

from os.path import expanduser

from std_msgs.msg import Bool
from std_msgs.msg import String

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

class ControlGuiPlugin(Plugin):
    enableMapUpdatesPublisher = rospy.Publisher('enableSpatialMapUpdates', Bool, queue_size=10)
    clearPublisher = rospy.Publisher('clearSpatialMap', Bool, queue_size=10)

    def __init__(self, context):
        super(ControlGuiPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ControlGuiPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('octomap_based_spatial_mapper_control_gui'), 'resource', 'ControlGui.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ControlGuiPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Register the callbacks.
        self._widget.enable_map_updates_button.clicked.connect(self.enable_map_updates_button_clicked)
        self._widget.disable_map_updates_button.clicked.connect(self.disable_map_updates_button_clicked)
        self._widget.clear_button.clicked.connect(self.clear_button_clicked)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.enableMapUpdatesPublisher.unregister()
        self.clearPublisher.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
    #     # Comment in to signal that the plugin has a way to configure
    #     # This will enable a setting button (gear icon) in each dock widget title bar
    #     # # Usually used to open a modal configuration dialog
    #     pass       

    def enable_map_updates_button_clicked(self, checked):
        rospy.loginfo('Enable map updates button clicked.')
        self.enableMapUpdatesPublisher.publish(True)

    def disable_map_updates_button_clicked(self, checked):
        rospy.loginfo('Disable map updates button clicked.')
        self.enableMapUpdatesPublisher.publish(False)

    def clear_button_clicked(self, checked):
        rospy.loginfo('Clear button clicked.')
        self.clearPublisher.publish(True)
