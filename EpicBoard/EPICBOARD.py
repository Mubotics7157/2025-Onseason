#FIX *** TOPICS

import ntcore
import serial
import time

#====================Dictionary Maps====================
#Serial Button Map Sheet: https://docs.google.com/spreadsheets/d/1YvCUePmxWHbtKLfM2LapoRFbz81KjYqjqmnQ18IcuiM/edit?usp=sharing

branch_to_button = {
    "a": "h", #Verified
    "b": "i", #Verified
    "c": "l", #Verified
    "d": "e", #Verified
    "e": "m", #Verified
    "f": "n", #Verified
    "g": "j", #Verified
    "h": "q", #Verified
    "i": "s", #Verified
    "j": "t", #Verified
    "k": "u", #Verified
    "l": "v", #Verified
}

level_to_button = {
    "NONE": "z", #***UNVERIFIED: Ensure "z" doesn't do anything
    "L1": "a", #Verified
    "L2": "b", #Verified
    "L3": "c", #Verified
    "L4": "d", #Verified
}

align_location_to_button = {
    "BARGE SHOT": "r",  #Verified
    "PROCESSOR": "f",  #Verified
    "CLIMBING": "g",  #Verified
}

#====================Main Loop Function Definition====================
def main():
    #====================NetworkTables Initialization====================
    ntcoreinst = ntcore.NetworkTableInstance.getDefault()

    #Creating connection parameters
    print("Establishing NetworkTables Client")
    ntcoreinst.startClient4("ButtonBoardPython")
    ntcoreinst.setServer("127.0.0.1")
    ntcoreinst.startDSClient()

    #Waiting for connection message
    print("PendingNetworkTables Server Access...")
    while not ntcoreinst.isConnected():
        time.sleep(0.1)
    print("NetworkTables Client Online!")

    #NetworkTable variables
    selected_reef_pole = ntcoreinst.getStringTopic("tagFollowing/selectedReefPoleChar").subscribe("none") #Selected FMS Branch Variable (a --> l)
    selected_level = ntcoreinst.getStringTopic("tagFollowing/selected level").subscribe("none") #Selected Scoring Level Variable ("NONE", "L1", "L2", "L3", "L4")
    align_location = ntcoreinst.getStringTopic("tagFollowing/selected location").subscribe("none") #Selected Aligning Location Variable ("REEF", "BARGE", "PROCESSOR", "CLIMBING")
    algae_mode = ntcoreinst.getBooleanTopic("algae mode").subscribe(False) #Algae Mode Variable

    #Serial setup
    ser = serial.Serial('COM5', 112500)
    #Example 1: L1 ON Command === ser.write(b'A\r')
    #Example 2: L1 OFF Command === ser.write(b'a\r')
    #Example 3: RGB Strip Vibrant Red Command === ser.write(b'3\r')

    #====================Illumination Logic====================
    while True:
        print("reef pole", selected_reef_pole.get())
        print("selected level", selected_level.get())
        print("align location", align_location.get())
        print("algae mode", algae_mode.get())

        #***for all of these, maybe keep track of current level, if level changes turn of previous level, than turn on the new level

        #====================Reef Branch Logic====================
        pole = selected_reef_pole.get("a")
        print("branch button ON", branch_to_button[pole].capitalize())
        print("branch button OFF", branch_to_button[pole].lower())

        #====================Reef Scoring Level Logic====================
        level = selected_level.get("NONE")
        print("level button ON", level_to_button[level].capitalize())
        print("level button OFF", level_to_button[level].lower())

        #====================Scoring Location Logic====================
        align_loc = align_location.get("REEF")
        if not align_loc == "REEF":
            print("align location ON", align_location_to_button[align_loc].capitalize())
            print("align location OFF", align_location_to_button[align_loc].lower())

        #====================Algae Mode Logic====================
        algae_feeling = algae_mode.get("False")
        if algae_feeling == "True":
            ser.write(b'3\r')
        else:
            ser.write(b'4\r')

        time.sleep(0.1) #Prevents overlooping

main() #Calls main function