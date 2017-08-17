import json

with open("targets.json","rb") as f:
    build_data = json.load(f)

targets = build_data['targets']

configs = build_data['configs']
