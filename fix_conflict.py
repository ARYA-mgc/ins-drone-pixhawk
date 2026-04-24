import re

with open('README.md', 'r', encoding='utf-8') as f:
    text = f.read()

idx = text.find('## 📄 License')
if idx != -1:
    text = text[:idx] + """## 📄 License

MIT License

---

## 📷 Project Documentation & Logs

### Hardware Assembly & Flight Test
![Real Flight Test](image%201.webp)
![Hardware Assembly](21b3fa5f6b6047958690e62dde1b69a32b4ea150_2_767x1024.jpeg)

### Real Log Images
![Log Image 1](drones-09-00027-g013-550.jpg)
![Log Image 2](drones-09-00405-g014-550.jpg)
"""

with open('README.md', 'w', encoding='utf-8') as f:
    f.write(text)
