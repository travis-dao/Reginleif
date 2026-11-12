import re

path = "build/Reginleif.map"
symbols = {}

with open(path) as f:
    for line in f:
        m = re.match(
            r"\s*0x([0-9a-fA-F]+)\s+0x([0-9a-fA-F]+)\s+(.+)",
            line,
        )
        if m:
            address = int(m.group(1), 16)
            size = int(m.group(2), 16)
            name = m.group(3).strip()

            if size >= 1024:
                symbols[(address, size, name)] = True

for address, size, name in sorted(
    symbols, key=lambda x: x[1], reverse=True
)[:40]:
    print(f"{size:10,d} bytes  0x{address:08x}  {name}")