import subprocess
import os,signal,time,sys
from rosbags.rosbag2 import Reader as ros2Reader
from rosbags.rosbag1 import Reader as ros1Reader
from rosbag2_interfaces.srv import *
import rclpy
from rclpy.node import Node


class bagfile_helper:
    def __init__(self):
        rclpy.init(args=None)
        self.Node_param = Node("bagRec")
        self.pause_service = self.Node_param.create_client(Pause,'/rosbag2_player/pause')
        self.subprocess_list =dict({})
        print("init done")

    def get_time_duration(self,bag_file : str):
        """
        Returns the time in inti for the latest bag file recorded 
        in location where the bag files are stored
        """
        try:
            direChoosen = os.path.dirname(bag_file)
            with ros2Reader(direChoosen) as reader:
                duration = round(reader.duration * pow(10,-9),2)
                start_time = reader.start_time
            return (duration,start_time)
        except Exception as e:
            print("[ros2][get_time_duration]Exception while trying to get_time_duration from bag file:"+str(e))
            return (0,0)

    """
    ros2 bag recorder
    def record_bag(self)-> None:
        '''
        Assuming the directory created at the initlisation of class is not delted
        '''
        try:
            print("[ros2][record_bag]"+str(self.bagfile_loaction)+" "+str(self.toggleROS))
            os.chdir(self.bagfile_loaction)
            command = ['ros2','bag','record','-a']
            self.subprocess_list["recorder"] = subprocess.Popen(command)
        except Exception as e:
            print("[ros2][record_bag]Exception while trying to record bag file:"+str(e))    
    """
        
    def play_recording(self,bag_file : str)-> None:
        """
        Play the latest bag file
        Assumption : multiple bag files are not being played
        """
        try:
            command = ['ros2','bag','play']
            command.append(bag_file)
            process_id = subprocess.Popen(command)
            self.subprocess_list["player"] = {}
            self.subprocess_list["player"]["process"]= process_id
            self.subprocess_list["player"]["timestamp"]=int(time.time())


        except Exception as e:
            print("[ros2][play_bag]Exception while trying to playing the  bag file:"+str(e))
        
    def stop_recording(self) -> bool:
        """
        Returns True after killing killing the subprocess
        Returns False if there is no active recording or in exception
        """
        try:
            print("[ros2][stop_recording]",self.subprocess_list)
            if "recorder" in self.subprocess_list:
                print("[ros2][stop_recording]killing the recorded")
                process = self.subprocess_list["recorder"]
                command = ['kill','-2',str(process.pid)]
                subprocess.run(command)
                self.subprocess_list.pop("recorder")
                return True
            else:
                return False
        except Exception as e:
            print("[ros2][stop_recording]Exception while trying to stop active bag file:"+str(e))
            return False

    def stop_player(self) -> bool:
        '''
        kill the player process and remove from the list
        '''
        try:
            if "player" in self.subprocess_list:
                if self.pause_service.service_is_ready():
                    command = ['kill','-2',str(self.subprocess_list["player"]["process"].pid)]
                    subprocess.run(command)
                    self.subprocess_list.pop("player")
        except Exception as e:
            print("[ros2][stop_player]Exception while stopping the playter "+str(e))
    
    def pause_playback(self) -> None:
        
        try:
            if "player" in self.subprocess_list:
                '''
                Implement checking played time with respcet to time in bag file and then pause
                '''
                if self.pause_service.service_is_ready():
                    command = ['ros2','service','call','/rosbag2_player/pause','rosbag2_interfaces/srv/Pause']
                    subprocess.run(command)
                else :
                    print("[ros2][pause_playback] Pause Service not available")
            else:
                print("[ros2][pause_playback] Player is not active")
        except Exception as e:
            print("[ros2][pause_playback]Exception while trying to stop active bag file:"+str(e))
    
    def playnextmsg(self) -> None:
        try:
            if "player" in self.subprocess_list and self.toggleROS == False:
                command = ['ros2','service','call','/rosbag2_player/play_next','rosbag2_interfaces/srv/PlayNext']
                subprocess.run(command)

        except Exception as e:
            print("[ros2][playnextmsg]Exception while trying play next msg:"+str(e))


    def resume_playback(self,bag_file : str,seek_position :int) -> None:
        try:
            if "player" in self.subprocess_list:
                if self.pause_service.service_is_ready():
                    command = ['ros2','service','call','/rosbag2_player/resume','rosbag2_interfaces/srv/Resume']
                    print("[ros2][resume_playback]",self.subprocess_list)
                    subprocess.run(command)
                else :
                    print("[ros2][resume_playback] Resume Service not available")
            else:
                print("[ros2][resume_playback] Player is not active")

        except Exception as e:
            print("[ros2][resume_playback]Exception while trying to resume_playback :"+str(e))
    
    def seek_playback(self,timestamp : int,bagfile :str,paused :bool = False) -> None:
        try:
            duration,start_time = self.get_time_duration(bagfile)
            if timestamp < duration:
                print("[ros2][seek_playback]time to seek start time")
                if "player" in self.subprocess_list:
                    if self.pause_service.service_is_ready():
                        command = ['kill','-2',str(self.subprocess_list["player"]["process"].pid)]
                        subprocess.run(command)
                        time.sleep(0.2)
                        if paused:
                            if timestamp != 0: 
                                command = ['ros2','bag','play',"--start-paused","--start-offset", str(timestamp),bagfile]
                            else:
                                command = ['ros2','bag','play',"--start-paused",bagfile]
                        else:
                            if timestamp != 0: 
                                command = ['ros2','bag','play',"--start-offset", str(timestamp),bagfile]
                            else :
                                command = ['ros2','bag','play',bagfile]
                        self.subprocess_list["player"]["process"] = subprocess.Popen(command)
                        self.subprocess_list["player"]["timestamp"] = int(time.time())
                    else :
                        print("[ros2][seek_playback] Seek_playback Service not present")

        except Exception as e:
            print("[ros2][seek_playback]Exception while trying to seeking playback :"+str(e))
