import os
from dotenv import load_dotenv

def environment(dotEnv=load_dotenv()):
    constants={
        "protocol":"https://",
        "baseUrl":os.getenv("BASE_URL"),
        "accessKey":os.getenv("API_KEY"),
        "LAT":os.getenv("LAT"),
        "LONG":os.getenv("LONG")
    }

    return constants