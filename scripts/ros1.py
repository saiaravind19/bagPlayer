import subprocess
import os,signal,time,sys
from rosbags.rosbag2 import Reader as ros2Reader
from rosbags.rosbag1 import Reader as ros1Reader
#from rosbag2_interfaces.srv import *
import rclpy
from rclpy.node import Node


class bagfile_helper:
    def __init__(self,absolute_path : str,isSim : bool ):
        """
        Note: Make sure to flush the bag director befre switch from ros2 to ros1 or viceversa
        Reason : naming scheme of ros2 and ros1 are different 
        """
        if not os.path.exists(absolute_path):   # used to cretae the directory of not exitsted 
            os.mkdir(absolute_path)
            print("[ros1][__inti__]creating directory",absolute_path)
        print("[ros1][__inti__]",absolute_path)
        self.toggleROS = isSim

        #if not self.toggleROS :
        #    rclpy.init(args=None)
        #    self.Node_param = Node("bagRec")
        #    #self.seek_service = self.Node_param.create_client(Seek,'/rosbag2_player/seek')
        #    self.pause_service = self.Node_param.create_client(Pause,'/rosbag2_player/pause')
        #    #self.resume_service = self.Node_param.create_client(Seek,'/rosbag2_player/resume')
        self.bagfile_loaction = absolute_path
        self.subprocess_list =dict({})

    def get_time_duration(self,bag_file : str) -> float :
        """
        Returns the time in inti for the latest bag file recorded 
        in location where the bag files are stored
        """
        try:
            with ros1Reader(bag_file) as reader:
                duration = round(reader.duration * pow(10,-9),2)
                start_time = reader.start_time
            
            return (duration,start_time)
        except Exception as e:
            print("[ros1][get_time_duration]Exception while trying to get_time_duration from bag file:"+str(e))
            return (0,0)

    def record_bag(self)-> None:
        """
        Assuming the directory created at the initlisation of class is not delted

        """
        try:
            print("[ros1][record_bag]",self.bagfile_loaction,self.toggleROS)
            os.chdir(self.bagfile_loaction)
            if not self.toggleROS:
                command = ['ros2','bag','record','-a']
                self.subprocess_list["recorder"] = subprocess.Popen(command)
            else:
                command = ['rosbag','record','-a']
                self.subprocess_list["recorder"] = subprocess.Popen(command)
        except Exception as e:
            print("[ros1][record_bag]Exception while trying to record bag file:"+str(e))
        
    def play_recording(self,bag_file : str)-> None:
        """
        Play the latest bag file
        Assumption : multiple bag files are not being played
        """
        try:
            if not self.toggleROS:
                command = ['ros2','bag','play']
                command.append(bag_file)
                process_id = subprocess.Popen(command)
                self.subprocess_list["player"] = {}
                self.subprocess_list["player"]["process"]= process_id
                self.subprocess_list["player"]["timestamp"]=int(time.time())

            else:
                command = ['rosbag','play']
                command.append(bag_file)
                process_id = subprocess.Popen(command)
                self.subprocess_list["player"] = {}
                self.subprocess_list["player"]["process"]= process_id
                self.subprocess_list["player"]["timestamp"]=int(time.time())

        except Exception as e:
            print("[ros1][play_bag]Exception while trying to playing the  bag file:"+str(e))
        
    def stop_recording(self) -> bool:
        """
        Returns True after killing killing the subprocess
        Returns False if there is no active recording or in exception
        """
        try:
            print("[ros1][record_bag]",self.subprocess_list)
            if "recorder" in self.subprocess_list:
                print("killing the recorded")
                process = self.subprocess_list["recorder"]
                command = ['kill','-2',str(process.pid)]
                subprocess.run(command)
                self.subprocess_list.pop("recorder")
                return True
            else:
                return False
        except Exception as e:
            print("[ros1][stop_recording]Exception while trying to stop active bag file:"+str(e))
            return False

    def stop_player(self) -> bool:
        '''
        kill the player process and remove from the list
        '''
        try:
            if "player" in self.subprocess_list:
                command = ['kill','-2',str(self.subprocess_list["player"]["process"].pid)]
                subprocess.run(command)
                self.subprocess_list.pop("player")
        except Exception as e:
            print("[ros1][stop_player]Exception while stopping the playter "+str(e))
    
    def pause_playback(self) -> None:
        
        try:
            if "player" in self.subprocess_list:
                '''
                Implement checking played time with respcet to time in bag file and then pause
                '''
                if not self.toggleROS:
                    if self.pause_service.service_is_ready():
                        command = ['ros2','service','call','/rosbag2_player/pause','rosbag2_interfaces/srv/Pause']
                        subprocess.run(command)
                    else :
                        print("[ros1][pause_playback] Pause Service not available")

                else :
                    print("[ros1][pause_playback] ",self.subprocess_list)
                    command = ['kill','-2',str(self.subprocess_list["player"]["process"].pid)]
                    subprocess.run(command)
            else:
                print("[ros1][pause_playback] Player is not active")
        except Exception as e:
            print("[ros1][pause_playback]Exception while trying to stop active bag file:"+str(e))
    
    def playnextmsg(self) -> None:
        try:
            if "player" in self.subprocess_list and self.toggleROS == False:
                command = ['ros2','service','call','/rosbag2_player/play_next','rosbag2_interfaces/srv/PlayNext']
                subprocess.run(command)

        except Exception as e:
            print("[ros1][playnextmsg]Exception while trying play next msg:"+str(e))


    def resume_playback(self,bag_file : str,seek_position :int) -> None:
        try:
            if "player" in self.subprocess_list:
                if not self.toggleROS:
                    if self.pause_service.service_is_ready():
                        command = ['ros2','service','call','/rosbag2_player/resume','rosbag2_interfaces/srv/Resume']
                        print("[ros1][resume_playback]",self.subprocess_list)
                        subprocess.run(command)
                    else :
                        print("[ros1][resume_playback] Resume Service not available")

                else :
                    command = ['kill','-2',str(self.subprocess_list["player"]["process"].pid)]
                    subprocess.run(command)
                    
                    if seek_position !=0:
                        command = ['rosbag','play','-s',str(seek_position)]
                        command.append(bag_file)
                    else:
                        command = ['rosbag','play']
                        command.append(bag_file)
                    self.subprocess_list["player"]["process"] = subprocess.Popen(command)
                    self.subprocess_list["player"]["timestamp"] = int(time.time())
            else:
                print("[ros1][resume_playback] Player is not active")

        except Exception as e:
            print("[ros1][resume_playback]Exception while trying to resume_playback :"+str(e))
    
    def seek_playback(self,timestamp : int,bagfile :str,paused :bool = False) -> None:
        try:
            duration,start_time = self.get_time_duration(bagfile)
            if timestamp < duration:
                print("[ros1][seek_playback]time to seek start time")
                if "player" in self.subprocess_list:
                    if not self.toggleROS: 
                        if self.pause_service.service_is_ready():
                            command = ['kill','-2',str(self.subprocess_list["player"]["process"].pid)]
                            subprocess.run(command)
                            
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
                            print("[ros1][seek_playback] Seek_playback Service not present")

                    else:
                        command = ['kill','-2',str(self.subprocess_list["player"]["process"].pid)]
                        subprocess.run(command)
                        
                        if paused:
                            if timestamp != 0: 
                                command = ['rosbag','play',"--pause","-s", str(timestamp),bagfile]
                                self.subprocess_list["player"]["timestamp"] = int(time.time()) - timestamp
                            else:
                                command = ['rosbag','play',"--pause",bagfile]
                                self.subprocess_list["player"]["timestamp"] = int(time.time())
                        else:
                            if timestamp != 0: 
                                command = ['rosbag','play','-s',str(timestamp),bagfile]
                                self.subprocess_list["player"]["timestamp"] = int(time.time()) - timestamp
                            else :
                                command = ['rosbag','play',bagfile]

                        self.subprocess_list["player"]["process"] = subprocess.Popen(command)
                        
        except Exception as e:
            print("[ros1][seek_playback]Exception while trying to seeking playback :"+str(e))

def runbagHelper(abs_path,isSim):
    if not isSim :
        recorder = bagfile_helper(absolute_path= abs_path,is_SITL = isSim)
    else :
        recorder = bagfile_helper(absolute_path= abs_path,is_SITL = isSim)
    
    return recorder
