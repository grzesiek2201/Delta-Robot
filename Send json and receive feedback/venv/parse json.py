import json

x = [{
    "name": "ArduinoJson",
    "stargazers": {"totalCount": 5246},
    "issues": {"totalCount": 15},
    },
    {
    "name": "pdfium-binaries",
    "stargazers": {"totalCount": 249},
    "issues": {"totalCount": 12},
     }
]

print(x)
y = json.dump(x)
print(y)