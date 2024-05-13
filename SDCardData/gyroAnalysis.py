import re
from openpyxl import Workbook
import matplotlib.pyplot as plt

def extract_numbers_from_text(text):
    """Extracts numbers from a given text using regular expressions."""
    numbers = re.findall(r':\s*(-?\d+\.*\d*)\n', text)
    return [float(num) for num in numbers]

def create_graph(numbers):
    """Creates a graph from the extracted numbers."""
    plt.plot(numbers)
    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.title('Extracted Numbers')
    plt.grid(True)
    plt.show()


# Read text file
with open('GYRO - Copy.txt', 'r') as file:
    text = file.read()

# Extract numbers
numbers = extract_numbers_from_text(text)

# Create Excel workbook and sheet
wb = Workbook()
ws = wb.active
ws.title = "Numbers"

# Write numbers to Excel sheet
row = 1
column = 1
for number in numbers:
    ws.cell(row=row, column=column, value=number)
    if column == 3:
        row += 1
        column = 1
    else:
        column += 1

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

