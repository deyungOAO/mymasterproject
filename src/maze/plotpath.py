import pandas as pd
import matplotlib.pyplot as plt

# Read CSV (assumes two columns: x, y)
df = pd.read_csv("coordinates.csv")

plt.plot(df.iloc[:, 0], df.iloc[:, 1], '-o')
plt.scatter(df.iloc[0, 0], df.iloc[0, 1], c='g', label='Start')
plt.scatter(df.iloc[-1, 0], df.iloc[-1, 1], c='r', label='End')
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.show()
