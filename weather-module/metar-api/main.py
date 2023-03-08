import os
import requests
from dotenv import load_dotenv

dotEnv=load_dotenv()
accessKey=os.getenv('API_KEY')
icao=os.getenv('ICAO')
endpoint=f'/metar/{icao}/decoded'

url = os.getenv('BASE_URL') + endpoint
response = requests.request("GET",url,headers={
    "X-API-Key":accessKey
})

print(response.text)