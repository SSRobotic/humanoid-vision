import os
from openai import OpenAI
import base64
client = OpenAI(
    api_key = "901cxxxxxxxxxxxxxxxxfa",      #key
    base_url = "https://xxxxxxxxxx/api/v3",  #大模型接口
)
with open("mmy.jpeg", 'rb') as image_file:
    image = 'data:image/jpeg;base64,' + base64.b64encode(image_file.read()).decode('utf-8')
# Image input:
response = client.chat.completions.create(
    model="ep-20250113182844-mbqsm",
    messages=[
        {
            "role": "user",
            "content": [
                {"type": "text", "text": "描述词？"},
                {
                    "type": "image_url",
                    "image_url": {
                        "url": image
                    }
                },
            ],
        }
    ],
)

print(response.choices[0])
