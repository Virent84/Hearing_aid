import json, os, copy

def load(path=None):
    path = path or os.environ.get("HA_CONFIG") or os.path.join(os.path.dirname(__file__), "..", "config.json")
    with open(path) as f:
        return json.load(f)

def deep_update(a, b):
    a = copy.deepcopy(a)
    for k, v in b.items():
        a[k] = deep_update(a[k], v) if isinstance(v, dict) and isinstance(a.get(k), dict) else v
    return a
