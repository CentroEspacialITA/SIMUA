import env
import requests
import json

def callAPI():

    # loading environment
    constants = env.environment()
    
    # creating URL
    endpoint = f"/metar/{constants['icao']}/decoded"
    url = constants["protocol"] + constants["baseUrl"] + endpoint

    # calling API
    method = "GET"
    headers = {"X-API-Key":constants["accessKey"]}
    response = requests.request(
        method,
        url,
        headers=headers
    )
    createdObject = json.loads(response.text)
    dumppedObject = json.dumps(createdObject,indent=4,ensure_ascii=False)

    print('\n',dumppedObject,'\n')

    return createdObject