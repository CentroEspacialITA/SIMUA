from rplidar import RPLidar
import time
import json
import os

class Lidar():
    def __init__(self):
        super(Lidar,self).__init__()
    
    @classmethod
    def lidarParams(cls):
        """
        The parameters DMAX, IMIN, and IMAX in your script are used to define certain properties of the plot and how the lidar scan data is visualized:

            - dmax: This parameter defines the maximum value for the radial distance in the polar plot. It determines how far from the center of the plot the lidar points can be plotted. In your script, it's set to 4000, which means that points with a distance greater than 4000 units from the center won't be shown in the plot.
            - imin: This parameter defines the minimum intensity value for the lidar points. Intensity is a measure of the strength of the returned laser signal. Setting IMIN to 0 means that points with intensity values less than 0 won't be shown in the plot.
            - imax: This parameter defines the maximum intensity value for the lidar points. In your script, it's set to 50. Points with intensity values greater than 50 won't be shown in the plot.

        These parameters are used to filter and visualize the lidar scan data. Points that don't meet the distance and intensity criteria are excluded from the plot. 
        By adjusting these parameters, you can control which data points are shown in the plot and how they are colored.
        """
        params={
            "port-name":"/dev/ttyUSB0",
            "dmax":4000,
            "imin":0,
            "imax":100
        }
        lidar=RPLidar(params["port-name"])
        lidar.clean_input()
        
        print(f"""
           _____ __    ___    __  _____________________
          / ___// /   /   |  /  |/  /_  __/ ____/ ____/
          \__ \/ /   / /| | / /|_/ / / / / __/ / /     
         ___/ / /___/ ___ |/ /  / / / / / /___/ /___   
        /____/_____/_/  |_/_/  /_/ /_/ /_____/\____/   
                                                    RPLIDAR
                                             
        [lidar info]
            # model: {lidar.get_info()['model']}
            # firmware: {lidar.get_info()['firmware']}
            # hardware: {lidar.get_info()['hardware']}
            # serial-no: {lidar.get_info()['serialnumber']}
            # status: {lidar.get_health()}
        """)
        
        lidar.start_motor()
        return lidar,params
    
    @classmethod
    def getLidarData(cls,saveInDisk=True,enableROS=True,renderPlot=False):
        """
        Args:
        saveInDisk: if True it generates a json file as output from the fetched Data
        enableROS: if True it enables the ROS service as a msg publisher
        renderPlot: if True it shows an animated radar plot with the mapped points cloud (2D)
        
        1. iter_measures() method:
        Yields
        ------
        - new_scan : bool
            True if measures belongs to a new scan
        - quality : int
            Reflected laser pulse strength
        - angle : float
            The measure heading angle in degree unit [0, 360)
        - distance : float
            Measured object distance related to the sensor's rotation center.
            In millimeter unit. Set to 0 when measure is invalid.
        
        2. iter_scans() method:
        Yields
        ------
        - scan : list
            List of the measures. Each measurment is tuple with following
            format: (quality, angle, distance). For values description please
            refer to `iter_measures` method's documentation.
        """
        
        lidar,_=cls.lidarParams()
        old_t = None
        valuesList=[]
        data={
                'new_scan_flag':[],
                'laser_pulse_strength':[],
                'angle_dg':[],
                'distance_mm':[],
                'frequency_hz':[],
                'rotation_rpm':[]
                }
        try:
            for m in lidar.iter_measures():
                
                now=time.time()
                
                # gets lidar mapping data
                data['new_scan_flag'].append(m[0])
                data['laser_pulse_strength'].append(m[1])
                data['angle_dg'].append(round(m[2],3))
                data['distance_mm'].append(round(m[3],3))

                # gets frequency and velocity
                if old_t is None:
                    old_t=now
                    continue
                delta=now-old_t
                data['frequency_hz'].append(round(1/delta,3))
                data['rotation_rpm'].append(round(60/delta,3))
                old_t=now

        except KeyboardInterrupt:
            print('\nStoping...\n')
        
        if saveInDisk==True:
            # converts data object to JSON format and saves into a file
            path='./output'
            os.makedirs(path,exist_ok=True)
            jsonFilePath=os.path.join(path, "lidar-data.json")
            with open(jsonFilePath, "w") as json_file:
                json.dump(data, json_file, indent=4)
            print(f"\nJSON file saved at: {jsonFilePath}\n")
        
        lidar.stop()
        lidar.disconnect()
                
Lidar.getLidarData()