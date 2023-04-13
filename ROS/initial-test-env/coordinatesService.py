import requests
import json

# Define the endpoint URL and API key
url = "https://www.googleapis.com/geolocation/v1/geolocate?key=YOUR_API_KEY"

# Send a POST request to the endpoint with empty body
response = requests.post(url)

# Parse the JSON response and extract the latitude and longitude
response_json = json.loads(response.text)
latitude = response_json['location']['lat']
longitude = response_json['location']['lng']
