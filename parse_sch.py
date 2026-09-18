import re
txt = open('pcbs/src/sensor_board.txt', encoding='utf-8').read()
nets = re.findall(r'<net name="([^"]+)"[^>]*>(.*?)</net>', txt, re.DOTALL)
for name, body in nets:
    pins = re.findall(r'part="([^"]+)"[^/]*pin="([^"]+)"', body)
    print(f'{name}: ' + ', '.join(f'{p}.{p2}' for p, p2 in pins))
