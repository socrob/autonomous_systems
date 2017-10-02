#!/usr/bin/python

import rospy
import smach
import smach_ros
import std_msgs.msg
import copy


class loop_for(smach.State):
    '''
    This state will return 'loop' MAX-1 times.
    On the MAX execute, 'continue' is returned.
    '''
    def __init__(self, MAX, sleep_time=0.0):
        smach.State.__init__(self, outcomes=['loop', 'continue'])
        self.max_loop_count = MAX
        self.loop_count = 0
        self.sleep_time = sleep_time

    def execute(self, foo):
        if self.loop_count < self.max_loop_count:
            rospy.sleep(self.sleep_time)
            rospy.loginfo('run number: %d' % self.loop_count)
            self.loop_count = self.loop_count + 1
            return 'loop'
        else:
            return 'continue'


class send_and_wait_events_combined(smach.State):
    """
    Send events to nodes (on event_in topic) and wait for responses from the nodes (on event_out topic)
    Sample usage:

    smach.StateMachine.add('START_DIRECT_BASE_CONTROLLER', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_navigation/direct_base_controller/event_in','e_start')],
                event_out_list=[
                        # if this happens it has succeeded
                        ('/mcr_navigation/direct_base_controller/event_out','e_success', True),
                        # if this happens it has failed
                        ('/mcr_navigation/collision_velocity_filter/event_out', 'e_zero_velocities_forwarded', False)]
                timeout_duration=20),
                transitions={'success':'SET_ACTION_LIB_SUCCESS',
                             'timeout':'SET_ACTION_LIB_FAILURE',
                             'failure':'SET_ACTION_LIB_FAILURE'})

    """
    def __init__(self, event_in_list=[], event_out_list=[], timeout_duration=20):
        smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'])

        if not(self.send_event_init(event_in_list)):
            exit()

        if not(self.wait_event_init(event_out_list, timeout_duration)):
            exit()

    def send_event_init(self, event_in_list):
        '''
        This method will take a list of events as input. Which are pair
        of name and value to publish. Publishers are created for each pair.
        '''
        self.event_in_publishers_list = []

        if len(event_in_list) == 0:
            rospy.logdebug('There are no events specified to publish.')
            return True

        self.event_in_msgs_ = []
        self.event_in_names_ = []
        self.possible_event_in_values = ['e_start', 'e_stop', 'e_trigger']
        for event_in in event_in_list:
            if len(event_in) != 2:
                rospy.logerr('The event list is malformed!!')
                return False
            elif event_in[1].lower() not in self.possible_event_in_values:
                rospy.logerr('Improper event value!!')
                return False

            event_in_name = event_in[0]
            self.event_in_names_.append(event_in_name)
            self.event_in_msgs_.append(event_in[1].lower())
            self.event_in_publishers_list.append(rospy.Publisher(event_in_name, std_msgs.msg.String, queue_size=1))

        return True

    def wait_event_init(self, event_out_list, timeout_duration):
        '''

        This method will take a list of event as input and then split them
        into positive events and negative events based on the desired action to take.
        Postive events return success when the corresponing component
        return succeeds whereas Negative events have define opposite
        behavior i.e. return failure when corresponding component returns success.

        Events specification:
           [(topic name, expected message, desired behavior),
            (topic name, expected message, desired behavior),
            .........
            .........
           ]

        topic name: string type
        expected message: string type
        desired behavior: True(positive events) and False(Negative events)

        State Behavior:
           returns success:
                   if all the positive events get expected values. or
           returns failure:
                   if any of the positive event get failure. or
                   if any of the negative event get success.       or
           retrurns timeout:
                   if all of the positive events or any of the negative event do not respond
                   with in the specified timeout for the state.
        '''
        self.event_out_subscribers_list_ = []
        self.timeout = rospy.Duration.from_sec(timeout_duration)
        if not self.init_state(event_out_list):
            rospy.logerr('Initialization of event out list failed.')
            return False
        return True

    def init_state(self, event_out_list):
        if len(event_out_list) == 0:
            rospy.logdebug('There are no events specified to subscribe.')
            return True

        for event_out in event_out_list:
            if len(event_out) != 3:
                rospy.logerr('Each specified event must contain topic name,expected message and desired behavior.')
                return False
            else:
                self.event_out_subscribers_list_.append(wait_for_single_event(event_out))
        return True

    def send_events(self, userdata):
        for index in range(len(self.event_in_publishers_list)):
            self.event_in_publishers_list[index].publish(self.event_in_msgs_[index])
            rospy.logdebug('Published the event_name: %s event_value: %s', self.event_in_names_[index],
                           self.event_in_msgs_[index])

        return 'success'

    def wait_for_events(self, userdata):
        start_time = rospy.Time.now()
        temp_events = copy.copy(self.event_out_subscribers_list_)

        while(rospy.Time.now() - start_time < self.timeout):

            num_pos_events = 0
            for event in temp_events:
                if event.get_event_behavior():
                    num_pos_events += 1
            if num_pos_events == 0:
                return 'success'

            for event in temp_events:
                result = event.getResult().lower()
                event_behavior = event.get_event_behavior()
                if result == "failure":
                    rospy.logerr("[wait_for_events] Received event {0}:{1}".format(event.get_event_name(),
                                 event.get_latest_event()))
                    return "failure"
                elif result == "success":
                    if not event_behavior:
                        rospy.logerr("[wait_for_events] Received event {0}:{1}".format(event.get_event_name(),
                                     event.get_latest_event()))
                        return "failure"
                    else:
                        rospy.loginfo("[wait_for_events] Received event {0}:{1}".format(event.get_event_name(),
                                      event.get_latest_event()))

                    temp_events.remove(event)

            rospy.sleep(0.01)

        return 'timeout'

    def reset_wait_events(self):
        for event_out in self.event_out_subscribers_list_:
            event_out.reset()

    def execute(self, userdata):

        return_msg = 'failure'

        self.reset_wait_events()

        if len(self.event_in_publishers_list) != 0:
            return_msg = self.send_events(userdata)

        if len(self.event_out_subscribers_list_) != 0:
            return_msg = self.wait_for_events(userdata)

        return return_msg


class send_event(smach.State):
    '''
    This state will take a list of event as input. Which are pair of name and value to publish.
    Output of this node is to publish the value in the provided topic name.
    '''

    def __init__(self, event_list):
        smach.State.__init__(self, outcomes=['success'])
        self.event_publisher_list = []
        self.expected_return_values_ = []
        self.event_names_ = []
        self.possible_event_values = ['e_start', 'e_stop', 'e_trigger', 'e_forget']
        for event in event_list:
            if len(event) != 2:
                rospy.logerr('The event list is malformed!!')
                exit()
            elif event[1].lower() not in self.possible_event_values:
                rospy.logerr('Improper event value!!')
                exit()

            event_name = event[0]
            self.event_names_.append(event_name)
            self.expected_return_values_.append(event[1].lower())
            self.event_publisher_list.append(rospy.Publisher(event_name, std_msgs.msg.String, queue_size=1))

        # give some time for the publishers to register in the network
        rospy.sleep(0.1)

    def execute(self, userdata):
        for index in range(len(self.event_publisher_list)):
            self.event_publisher_list[index].publish(self.expected_return_values_[index])
            rospy.logdebug('Published the event_name: %s event_value: %s', self.event_names_[index],
                           self.expected_return_values_[index])
        return 'success'


class wait_for_single_event(smach.State):
    '''
    This state will take a event name as input and waits for the event to
    be published.
    '''
    def __init__(self, single_event):
        smach.State.__init__(self, outcomes=['success', 'failure', 'no_response'])
        self.event_name_ = single_event[0]
        self.expected_message_ = single_event[1]
        self.event_behavior = single_event[2]

        rospy.Subscriber(self.event_name_, std_msgs.msg.String, self.event_cb)
        self.callback_msg_ = None
        self.latest_event = ""

    def event_cb(self, callback_msg):
        self.callback_msg_ = callback_msg

    def reset(self):
        self.callback_msg_ = None

    def getResult(self):
        if self.callback_msg_ is None:
            return 'no_response'
        self.latest_event = self.callback_msg_.data

        if (self.callback_msg_.data == self.expected_message_):
            self.callback_msg_ = None
            return 'success'
        elif self.event_behavior:
            self.callback_msg_ = None
            return 'failure'

        return 'no_response'

    def get_event_name(self):
        return self.event_name_

    def get_event_behavior(self):
        return self.event_behavior

    def get_latest_event(self):
        return self.latest_event


class wait_for_events(smach.State):
    '''

    This state will take a list of event as input and then split them into positive events and negative events
    based on the desired action to take.
    Postive events return success when the corresponing component return succeeds whereas Negative events have
    define opposite behavior i.e. return failure when corresponding component returns success.

    Events specification:
       [(topic name, expected message, desired behavior),
        (topic name, expected message, desired behavior),
        .........
        .........
       ]

    topic name: string type
    expected message: string type
    desired behavior: True(positive events) and False(Negative events)

    State Behavior:
       returns success:
               if all the positive events get expected values. or
       returns failure:
               if any of the positive event get failure. or
               if any of the negative event get success.       or
       retrurns timeout:
               if all of the positive events or any of the negative event do not respond
               with in the specified timeout for the state.
    '''
    def __init__(self, event_list, timeout_duration=20):
        smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'])
        self.event_publisher_list = []
        self.events_ = []
        self.timeout = rospy.Duration.from_sec(timeout_duration)
        self.event_type = []
        if not self.init_state(event_list):
            rospy.logerr('[wait for events state] Initialization failed.')
            exit()

    def init_state(self, event_list):
        if len(event_list) == 0:
            rospy.logerr('[wait for events] There are no events specified to subscribe.')
            return False

        for event in event_list:
            if len(event) != 3:
                rospy.logerr('[wait for events] Each specified event must contain topic name,'
                             'expected message and desired behavior.')
                return False
            else:
                self.events_.append(wait_for_single_event(event))
        return True

    def execute(self, userdata):

        start_time = rospy.Time.now()
        temp_events = copy.copy(self.events_)

        while(rospy.Time.now() - start_time < self.timeout):
            num_pos_events = 0
            for event in temp_events:
                if event.get_event_behavior():
                    num_pos_events += 1
            if num_pos_events == 0:
                return 'success'

            for event in temp_events:
                result = event.getResult().lower()
                event_behavior = event.get_event_behavior()
                if result == "failure":
                    rospy.logerr("Received event {0}:{1}".format(event.get_event_name(), event.get_latest_event()))
                    return "failure"
                elif result == "success":
                    if not event_behavior:
                        rospy.logerr("Received event {0}:{1}".format(event.get_event_name(), event.get_latest_event()))
                        return "failure"
                    else:
                        rospy.loginfo("Received event {0}:{1}".format(event.get_event_name(), event.get_latest_event()))

                    temp_events.remove(event)

            rospy.sleep(0.01)

        return 'timeout'


class set_named_config(smach.State):
    def __init__(self, named_config):
        smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'])
        self.named_config = named_config
        self.config_name_pub = rospy.Publisher("/mcr_common/dynamic_reconfigure_client/configuration_name",
                                              std_msgs.msg.String, queue_size=1)
        self.event_in_pub = rospy.Publisher("/mcr_common/dynamic_reconfigure_client/event_in",
                                              std_msgs.msg.String, queue_size=1)
        self.event_out_sub = rospy.Subscriber("/mcr_common/dynamic_reconfigure_client/event_out",
                                              std_msgs.msg.String, self.event_cb)
        self.event = None
        # give some time for publishers to register in the network, (specially needed if this function is the first state)
        rospy.sleep(0.1)

    def event_cb(self, msg):
        self.event = msg.data

    def execute(self, userdata):
        self.event = None

        self.config_name_pub.publish(self.named_config)
        self.event_in_pub.publish("e_start")

        timeout = rospy.Duration.from_sec(1.0)
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) < timeout:
            if self.event:
                if self.event == "e_success":
                    return 'success'
                else:
                    return 'failure'
            rate.sleep()
        return 'timeout'
