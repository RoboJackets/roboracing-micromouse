# Command Gen Instructions

## Use Case

Use the `cmdgen` tool to vizualize what a string of commands should look like. Takes in string of "Moves" as defined in the code and outputs the resulting path and commands.

## Setup

To use the cmdgen tool, make sure platformio and its VSCode extension are installed. Under the `cmdgen` folder in the platformio extension in VSCode, under the General folder, press Build. It should say `cmdgen` and SUCCESS in the terminal.

Now, setup the python environment. Create your venv with `python3 -m venv .venv`. Then source with `source ./.venv/bin/activate`. Then install matplotlib, the only current dependency, with `pip install matplotlib`.

## Run

Finally, to run, the following command needs to be run: `echo MOVES | .pio/build/cmdgen/program | python3 tools/cmdviz.py`, where `MOVES` is some string of moves, e.g. `"FRFFRLLRFF"`

To step through how each input move is converted into commands, run
`python3 tools/cmdviz.py --moves FFRLRL`. Use the **Previous** and **Next**
buttons, or the left and right arrow keys, to move through the input. Each step
shows the commands emitted by that move and the path generated so far. The
generator may buffer moves until later instructions resolve them; a final
sequence without `S` can therefore have unconverted moves.