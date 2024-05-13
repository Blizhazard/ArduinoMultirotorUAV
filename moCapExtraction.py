import re
from openpyxl import Workbook
import matplotlib.pyplot as plt


def create_graph(numbers):
    """Creates a graph from the extracted numbers."""
    plt.plot(numbers)
    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.title('Extracted Numbers')
    plt.grid(True)
    plt.show()
    
    
    
    
# Create Excel workbook and sheet
wb = Workbook()
ws = wb.active
ws.title = "Numbers"
    
roll = []
pitch = []
yaw = []
count = 0
for col in ws.iter_cols(min_row=40000, max_col=3, max_row = 59473, values_only=True):
    count += 1
    if count == 1:
        for value in col:
            if abs(value) < 180:
                roll.append(value)
            elif abs(value) < 500:
                roll.append(value-360)
    elif count == 2:
        for value in col:
            if abs(value) < 500:
                pitch.append(value)
    else:
        for value in col:
            if abs(value) < 500:
                yaw.append(value)
        

create_graph(roll)
create_graph(pitch)
create_graph(yaw)

# Save Excel file
wb.save("output.xlsx")
print("Numbers extracted and saved to output.xlsx")