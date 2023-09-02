from methods import Lidar

if __name__ == '__main__':
    Lidar.getLidarData(
        saveInDisk=True, # saves a JSON from obtained data
        renderPlot=True # shows a radar plot from the mapped points
    )