from pathlib import Path
import subprocess
from tkinter import *

def run_maze_sim():
    sim = subprocess.Popen([str(Path(__file__).resolve().parent.parent) + "/bin/Maze", selected_algo_gen.get(), "astar" if selected_algo_solve.get() == "A*" else selected_algo_solve.get(), dim.get("1.0", END), interval.get("1.0", END), seed.get("1.0", END)], stdout=subprocess.PIPE, text=True)
    for line in sim.stdout:
        stdout.config(state=NORMAL)
        stdout.insert(END, line)
        stdout.config(state=DISABLED)
        stdout.see(END)
        stdout.update()
        if line.strip() == "SOLVING COMPLETE":
            sim.stdout.close()
            root.update_idletasks()
            return

algos_gen = ['rdfs']
algos_solve = ['dfs', 'bfs', 'dfs_greedy', 'A*']

root = Tk()
root.title('maze_sim_launcher')
root.minsize(380,380)
root.maxsize(380,380)

frame = Frame(root)
frame.grid(row=9, column=2, pady=20, padx=20)

algo_gen_lab = Label(frame, text="Generation Algorithm: ")
algo_gen_lab.grid(row=0, column=0)
selected_algo_gen= StringVar()
selected_algo_gen.set(algos_gen[0])
algo_gen = OptionMenu(frame, selected_algo_gen, *algos_gen)
algo_gen.grid(row=1, column=0)

algo_solve_lab = Label(frame, text="Solver Algorithm: ")
algo_solve_lab.grid(row=2, column=0)
selected_algo_solve = StringVar()
selected_algo_solve.set(algos_solve[0])
algo_solve = OptionMenu(frame, selected_algo_solve, *algos_solve)
algo_solve.grid(row=3, column=0)

dim_lab = Label(frame, text="Dimensions: ")
dim_lab.grid(row=4, column=0)
dim = Text(frame, height=1, width=5)
dim.grid(row=5, column=0)

interval_lab = Label(frame, text="Interval (ms): ")
interval_lab.grid(row=6, column=0)
interval = Text(frame, height=1, width=5)
interval.grid(row=7, column=0)

seed_lab = Label(frame, text="Seed: ")
seed_lab.grid(row=8, column=0)
seed = Text(frame, height=1, width=5)
seed.grid(row=9, column=0)

launch = Button(frame, text="Launch!", command=run_maze_sim)
launch.grid(row=10, column=0)

stdout = Text(frame, height=23, width=25, state=DISABLED)
stdout.grid(row=0, column=1, rowspan=10)

root.mainloop()
