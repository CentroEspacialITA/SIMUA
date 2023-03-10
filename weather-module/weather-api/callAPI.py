import env
import requests
import json

def callAPI():
    """
    Pass US Zipcode, UK Postcode, Canada Postalcode, IP address, 
    Latitude/Longitude (decimal degree) or city name
    """

    # loading environment
    constants = env.environment()

    # creating URL
    endpoint = f"/v1/forecast.json?key={constants['accessKey']}&q={constants['LAT']},{constants['LONG']}&days=1&aqi=yes&alerts=yes"
    url = constants["protocol"] + constants["baseUrl"] + endpoint

    # calling API
    method = "GET"
    response = requests.request(
        method,
        url
    )
    createdObject = json.loads(response.text)
    dumppedObject = json.dumps(createdObject,indent=4,ensure_ascii=False)

    print('\n',dumppedObject,'\n')

    return createdObject
