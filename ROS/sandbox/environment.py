from dotenv import load_dotenv
import os

def env(dotEnv=load_dotenv()):
    constants = {
        'dataStreamer':os.getenv('TALKER_NODE_NAME'),
        'dataReceiver':os.getenv('LISTENER_NODE_NAME'),
        'protocolKey':os.getenv('ROS_DOMAIN_ID')
    }
    return constants