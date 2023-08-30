from rplidar import RPLidar
import time

class RPLidar():
    def __init__(self):
        super(RPLidar,self).__init__()
    
    @classmethod
    def lidarParams(self):
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
        lidar.start_motor()
        return lidar,params
    
    def getLidarData(self):
        
        lidar=self.lidarParams[0]

        data=[]
        old_t = None
        try:
            for _ in lidar.iter_scans():
                now = time.time()
                if old_t is None:
                    old_t = now
                    continue
                delta = now - old_t
        except:
            
