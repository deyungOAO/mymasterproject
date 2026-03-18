import subprocess

runs=10
counter=0
for i in range(runs):  # run 3 times
    print(f"Run {i+1}")
    subprocess.run(["python3", "buildmaze.py"])
    result = subprocess.run(
        ["python3", "A*final.py"],
        capture_output=True,
        text=True
    )
    
    if result.stdout.strip():  # if code2.py printed something
        counter += 1
        print(result.stdout, end="")  # show its output
    else:
        print("(code2.py printed nothing)")

print(f"\nSummary: code2.py printed something {counter} times out of {runs}")